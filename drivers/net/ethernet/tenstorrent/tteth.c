#include <linux/module.h>
#include <linux/pci.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/timer.h>
#include <linux/ip.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/ioport.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/io.h>

/*
Lot of this is based on linux/drivers/net/ethernet/ftmac100.c
*/

#define DRV_NAME "tteth"

#define MTU ETH_DATA_LEN // MTU (Ignoring ethernet header) that is set for adapter = 1500
#define PACKET_SIZE ETH_FRAME_LEN // MTU + ethernet header size = 1514

// Bottom 2 vars are customizable by user for improving perf
#define BUFFER_SIZE 500
#define POLL_INTERVAL (HZ / 1000) // Poll every 1ms

#define REGS 0x2ff10000UL

#define MAGIC 0x4D13ED8246A0f856ULL

struct tteth_eth_priv {
    struct net_device *netdev;
    struct device *dev;
    struct napi_struct napi;
    int irq;

    void __iomem *shmem;
    size_t shmem_size;
    spinlock_t send_lock;
    spinlock_t recv_lock;
    struct timer_list tteth_timer;
    struct resource *res;

};

struct packet{
    uint32_t len;
    char data[PACKET_SIZE];
};

// Represents shared memory region for one way communication
// Total shared memory size is 2* shared_data regions
// size of shared data
// 4 + 4 + 500*(4+1514)
// = 759008
// size of Total shared memory region
// = 2*size of shared data (one for tx, one for rx)
// = 1518016
// which is less than 2M TLb window (2097152)
struct one_side_buffer{
    uint32_t location_sent;
    uint32_t location_rcvd;
    struct packet buffer[BUFFER_SIZE];
};

struct shared_data{
    uint64_t magic;
    uint32_t interrupts;
    struct one_side_buffer recv;
    struct one_side_buffer send;
};

static void tteth_timer_callback(struct timer_list *t);
static irqreturn_t tteth_interrupt_handler(int irq, void *dev_id);

static int tteth_eth_open(struct net_device *netdev) {
    struct tteth_eth_priv *priv = netdev_priv(netdev);
    int err;

    if (priv->irq >=0){
        err = request_irq(priv->irq, tteth_interrupt_handler, 0, netdev->name, netdev);
        if (err) {
            netdev_err(netdev, "failed to request irq %d\n", priv->irq);
            goto err_irq;
        }
    }

    napi_enable(&priv->napi);
    netif_start_queue(netdev);

    if (priv->irq < 0){
        timer_setup(&(priv->tteth_timer), tteth_timer_callback, 0);
        mod_timer(&(priv->tteth_timer), jiffies + POLL_INTERVAL);
    }
    
    return 0;

err_irq:
    return err;
}

static int tteth_eth_stop(struct net_device *netdev) {
    struct tteth_eth_priv *priv = netdev_priv(netdev);

    if (priv->irq < 0)
        timer_delete_sync(&(priv->tteth_timer));

    netif_stop_queue(netdev);
    napi_disable(&priv->napi);
    if (priv->irq >=0)
        free_irq(priv->irq, netdev);
    return 0;
}

static irqreturn_t tteth_interrupt_handler(int irq, void *dev_id)
{
    struct net_device *netdev = dev_id;
	struct tteth_eth_priv *priv = netdev_priv(netdev);

    // // Does this disable interrupts?
    // // How to interrupts get enabled again?
    iowrite32(1, &(((struct shared_data*) (priv->shmem))->interrupts));

	if (likely(netif_running(netdev)))
		napi_schedule(&priv->napi);

	return IRQ_HANDLED;
}

static netdev_tx_t tteth_eth_start_xmit(struct sk_buff *skb, struct net_device *netdev) {
    struct tteth_eth_priv *priv = netdev_priv(netdev);
    uint32_t location_sent, location_rcvd;
    int len;
    struct packet *to_transmit;

    struct shared_data *shmem = (struct shared_data*) (priv->shmem);
    struct one_side_buffer *data = &(shmem->send);
    spin_lock(&(priv->send_lock));

    // Check if buffer has space to receive one packet
    // Return NETDEV_TX_BUSY if buffer is full
    // Disable transmissions
    if (((ioread32(&(data->location_sent)) + 1) % BUFFER_SIZE) == ioread32(&(data->location_rcvd))){
        pr_info("%s: tx dropped packet dev %s: len %d", DRV_NAME, netdev->name, skb->len);
        netdev->stats.tx_dropped++;
        netif_stop_queue(netdev);
        spin_unlock(&(priv->send_lock));
        return NETDEV_TX_BUSY;
    }

    location_sent = ioread32(&(data->location_sent));
    location_rcvd = ioread32(&(data->location_rcvd));
    smp_rmb();
    
    len = skb->len;
    to_transmit = (struct packet*) (data->buffer + location_sent);
    // smp_wmb();
    iowrite32(skb->len, &(to_transmit->len));

    // smp_mb();
    memcpy_toio(to_transmit->data, skb->data, len);
    // smp_mb();

    smp_wmb();
    iowrite32((location_sent + 1) % BUFFER_SIZE, &(data->location_sent));

    netdev->stats.tx_packets++;
    netdev->stats.tx_bytes += skb->len;
    
    spin_unlock(&(priv->send_lock));
    dev_kfree_skb(skb);
    return NETDEV_TX_OK;
}

static int tteth_eth_rx(struct napi_struct *napi, int budget) {
    struct tteth_eth_priv *priv = container_of(napi, struct tteth_eth_priv, napi);
    struct net_device *netdev = priv->netdev;
    uint32_t location_rcvd, location_sent;
    int len;
    struct packet *to_receive;
    struct sk_buff *skb;
    int rx_result;
    int work_done = 0;

    struct shared_data *shmem = (struct shared_data*) (priv->shmem);
    struct one_side_buffer *data = &(shmem->recv);

    spin_lock(&(priv->recv_lock));
    while (work_done < budget){
        // Check if there is data to receive
        if (ioread32(&(data->location_rcvd)) == ioread32(&(data->location_sent))) {
            break; // No data to receive
        }

        location_rcvd = ioread32(&(data->location_rcvd));
        location_sent = ioread32(&(data->location_sent));
        smp_rmb();

        to_receive = (struct packet*) (data->buffer + location_rcvd);
        len = ioread32(&(to_receive->len));
        // smp_rmb();

        skb = alloc_skb(len + 2, GFP_ATOMIC);
        if (!skb) {
            pr_err("Failed to allocate skb\n");
            break;
        }

        skb_reserve(skb, 2);
        skb_put(skb, len);

        // smp_mb();
        memcpy_fromio(skb->data, to_receive->data, len);
        // smp_mb();

        smp_wmb();
        iowrite32((location_rcvd + 1) % BUFFER_SIZE, &(data->location_rcvd));

        skb->dev = netdev;
        skb->protocol = eth_type_trans(skb, netdev);
        skb->ip_summed = CHECKSUM_UNNECESSARY;

        rx_result = netif_rx(skb);
        if (rx_result == NET_RX_DROP) {
            netdev->stats.rx_dropped++;
            pr_info("%s: rx dropped packet dev %s: len %d", DRV_NAME, netdev->name, len);
        } else {
            netdev->stats.rx_packets++;
            netdev->stats.rx_bytes += len;
        }
        work_done++;
    }
    spin_unlock(&(priv->recv_lock));

    if (work_done < budget){
        napi_complete(napi);
    }
    return work_done;
}

static void tteth_timer_callback(struct timer_list *t) {
    struct tteth_eth_priv *priv = from_timer(priv, t, tteth_timer);
    if (likely(netif_running(priv->netdev)))
		napi_schedule(&priv->napi);
    // Re-arm the timer
    mod_timer(&(priv->tteth_timer), jiffies + POLL_INTERVAL);
}

static int tteth_napi_poll(struct napi_struct *napi, int budget){
    struct tteth_eth_priv *priv = container_of(napi, struct tteth_eth_priv, napi);
    struct shared_data *shmem = (struct shared_data*) (priv->shmem);
    struct one_side_buffer *data = &(shmem->send);
    // Do RX
    int work_done = tteth_eth_rx(napi, budget);
    
    // If TX Buffer has space, enable transmissions again
    spin_lock(&(priv->send_lock));
    if ((((ioread32(&(data->location_sent)) + 1) % BUFFER_SIZE) != ioread32(&(data->location_rcvd))) && netif_queue_stopped(priv->netdev)){
        pr_info("%s: timer waking queue", DRV_NAME);
        netif_wake_queue(priv->netdev);
    }
    smp_rmb();
    spin_unlock(&(priv->send_lock));
    
    return work_done;
}

static const struct net_device_ops tteth_eth_netdev_ops = {
    .ndo_open = tteth_eth_open,
    .ndo_stop = tteth_eth_stop,
    .ndo_start_xmit = tteth_eth_start_xmit,
};

static int tteth_eth_probe(struct platform_device *pdev) {
    struct net_device *netdev;
    struct tteth_eth_priv *priv;
    struct resource *res;
    int irq;
    struct device *dev = &pdev->dev;
    int err;
    struct shared_data *buffer;

    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!res) {
        dev_err(dev, "Failed to get memory resource from reg property\n");
        return -ENXIO;
    }
    
    irq = platform_get_irq(pdev, 0);
	// If interrupts not available, go to polling mode
    if (irq < 0){
        dev_info(dev, "failed to get interrupts, going to polling mode");
    }
     
    netdev = alloc_etherdev(sizeof(*priv));
    if (!netdev){
        return -ENOMEM;
        goto err_alloc_etherdev;
    }
    
    SET_NETDEV_DEV(netdev, &pdev->dev);
    netdev->netdev_ops = &tteth_eth_netdev_ops;
    netdev->mtu = MTU;
    netdev->features &= ~NETIF_F_HW_CSUM; // Missing csum, tcp/udp forwarding doesn't work
    
    platform_set_drvdata(pdev, netdev);
    
    priv = netdev_priv(netdev);
    priv->netdev = netdev;
    priv->dev = &pdev->dev;
    priv->shmem_size = resource_size(res);
    
    spin_lock_init(&(priv->send_lock));
    spin_lock_init(&(priv->recv_lock));

    netif_napi_add(netdev, &priv->napi, tteth_napi_poll);
    

    // Request resource management
    priv->res = request_mem_region(res->start, priv->shmem_size, DRV_NAME);
    if (!priv->res) {
        dev_err(dev, "Failed to request memory region from 0x%08lx to 0x%08lx\n",
            (unsigned long)res->start, (unsigned long)(res->start + priv->shmem_size));
            err = -ENOMEM;
            goto err_req_mem;
    }

    // Map the reserved memory
    priv->shmem = ioremap(res->start, priv->shmem_size);
    if (!priv->shmem) {
        dev_err(dev, "Failed to map reserved memory\n");
        goto err_ioremap;
    }
    
    priv->irq = irq;
    
    // Zero out the buffer
    buffer = (struct shared_data*) (priv->shmem);
    memset_io(&(buffer->send), 0, sizeof(struct one_side_buffer));
    memset_io(&(buffer->recv), 0, sizeof(struct one_side_buffer));
    buffer->magic = MAGIC;
    buffer->interrupts = 0;

    
    err = register_netdev(netdev);
    if (err) {
        dev_err(&pdev->dev, "Failed to register netdev\n");
		goto err_register_netdev;
	}
    
    eth_hw_addr_random(netdev);

    return 0;


err_register_netdev:
	iounmap(priv->shmem);
err_ioremap:
	release_resource(priv->res);
err_req_mem:
	netif_napi_del(&priv->napi);
// defer_get_mac:
	free_netdev(netdev);
err_alloc_etherdev:
	return err;
}

static void tteth_eth_remove(struct platform_device *pdev) {
    struct net_device *netdev = platform_get_drvdata(pdev);
    struct tteth_eth_priv *priv = netdev_priv(netdev);

    unregister_netdev(netdev);
    iounmap(priv->shmem);
    release_resource(priv->res);
    
    netif_napi_del(&priv->napi);
    free_netdev(netdev);
}

static const struct of_device_id tteth_of_match[] = {
    { .compatible = "tenstorrent,ethernet" },
    { }
};

static struct platform_driver tteth_eth_driver = {
    .probe = tteth_eth_probe,
    .remove = tteth_eth_remove,
    .driver = {
        .name = DRV_NAME,
        .of_match_table = tteth_of_match,
    },
};

module_platform_driver(tteth_eth_driver);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Anirudh Srinivasan");
MODULE_DESCRIPTION("tteth Ethernet Driver");
MODULE_DEVICE_TABLE(of, tteth_of_match);
