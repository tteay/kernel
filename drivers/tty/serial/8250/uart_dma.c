
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/circ_buf.h>
#include <linux/serial_core.h>
#include <linux/serial_8250.h>
#include "uart_dma.h"

#define WARN_URDMA(fmt, args...) printk(KERN_WARNING"%s()L%u:*** " fmt "\n", __func__, __LINE__, ##args)
#define ERR_URDMA(fmt, args...) printk(KERN_ERR "%s()L%u: ***" fmt "\n", __func__, __LINE__, ##args)
#define INFO_URDMA(fmt, args...) printk(KERN_INFO "%s()L%u: " fmt "\n", __func__, __LINE__, ##args)

#if 0
#define DBG_URDMA(fmt, args...) printk(KERN_INFO "%s()L%u: " fmt "\n", __func__, __LINE__, ##args)
#else
#define DBG_URDMA(fmt, args...)
//#define INFO_URDMA(fmt, args...)
//#define WARN_URDMA(fmt, args...)
//#define ERR_URDMA(fmt, args...)
#endif

//#define DUMP_RECV_CHARS
#define DUMP_BEFORE_STOP
#define DUMP_AFTER_START

#define DUMP_DMA_STATE

#ifdef DUMP_DMA_STATE

void uart_dma_dump_tx_state(struct uart_dma_tx_chan *chan) {
	struct uart_dma_data *dma = chan->dma;
	volatile struct uart_dma_regs __iomem *base = dma->dma_base;
	volatile struct uart_dma_misc_regs __iomem *miscbase = dma->dma_miscbase;

	ERR_URDMA("uart_irqs "
					"0x%x",
					dma->dma_miscbase->uart_irqs);
	ERR_URDMA("line %d, "
				"tx_ctrl 0x%x@0x%p, "
				//"offset 0x%x, "
				"base/len/thres 0x%x/0x%x/0x%x, "
				"hw_rd/remain 0x%x/0x%x, "
				"clk_ctrl 0x%x@0x%p"
				,
		dma->port->line,
		base->tx_ctrl, &base->tx_ctrl,
		//(unsigned int) &((struct uart_dma_regs *)0)->tx_ctrl,
		base->tx_buf_addr, base->tx_buf_len, base->tx_buf_thres,
		base->tx_hw_rd_addr, base->tx_remain,
		miscbase->clk_ctrl, &miscbase->clk_ctrl
		);
}

void uart_dma_dump_rx_state(struct uart_dma_rx_chan *chan) {
	struct uart_dma_data *dma = chan->dma;
	struct circ_buf *circ = &chan->circ_buf;
	volatile struct uart_dma_regs __iomem *base = dma->dma_base;
	volatile struct uart_dma_misc_regs __iomem *miscbase = dma->dma_miscbase;
	
	ERR_URDMA("uart_irqs "
					"0x%x",
					dma->dma_miscbase->uart_irqs);
	ERR_URDMA("line %d, "
				"rx_ctrl 0x%x@0x%p, "
				//"offset 0x%x, "
				"base/len/thres 0x%x/0x%x/0x%x, "
				"tail/hw_wr/remain 0x%x/0x%x/0x%x, "
				"clk_ctrl 0x%x@0x%p, "
				"rx_tmout 0x%x"
				,
		dma->port->line,
		base->rx_ctrl, &base->rx_ctrl,
		//(unsigned int) &((struct uart_dma_regs *)0)->tx_ctrl,
		base->rx_buf_addr, base->rx_buf_len, base->rx_buf_thres_2,
		circ->tail+chan->dma_base_addr, base->rx_hw_wr_addr, base->rx_remain,
		miscbase->clk_ctrl, &miscbase->clk_ctrl,
		base->rx_timeout
		);
}
#if 0
struct uart_8250_port {
	struct uart_port	port;
	struct timer_list	timer;		/* "no irq" timer */
	struct list_head	list;		/* ports on this IRQ */
	unsigned short		capabilities;	/* port capabilities */
	unsigned short		bugs;		/* port bugs */
	unsigned int		tx_loadsz;	/* transmit fifo load size */
	unsigned char		acr;
	unsigned char		ier;
	unsigned char		lcr;
	unsigned char		mcr;
	unsigned char		mcr_mask;	/* mask of user bits */
	unsigned char		mcr_force;	/* mask of forced bits */
	unsigned char		cur_iotype;	/* Running I/O type */

	/*
	 * Some bits in registers are cleared on a read, so they must
	 * be saved whenever the register is read but the bits will not
	 * be immediately processed.
	 */
#define LSR_SAVE_FLAGS UART_LSR_BRK_ERROR_BITS
	unsigned char		lsr_saved_flags;
#define MSR_SAVE_FLAGS UART_MSR_ANY_DELTA
	unsigned char		msr_saved_flags;

	/*
	 * We provide a per-port pm hook.
	 */
	void			(*pm)(struct uart_port *port,
				      unsigned int state, unsigned int old);
};
#endif

extern struct uart_8250_port serial8250_ports[3];

static ssize_t uart_dma_dump_states(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
	unsigned long idx = 0;
	struct uart_8250_port *p;
	struct uart_dma_data *dma;

	printk(KERN_ERR "dev %p buf %s count %zu\n", dev, buf, count);
#if 0
	if(dev) {
		printk(KERN_ERR "dev-plat_p %p\n", dev->platform_data);
	}

	return count;
#endif
	idx = simple_strtoul(buf, NULL, 0);
	if(idx > 3) {
		printk(KERN_ERR "wrong port idx %lu\n", idx);
		return count;
	}
	p = &serial8250_ports[idx];
	//p += idx;
	if(!p) {
		printk(KERN_ERR "plat_serial8250_port 0x%p, port %lu\n", p, idx);
		return count;
	}
	dma = p->port.private_data;
	if(dma && dma->enable) {
		uart_dma_dump_tx_state(&dma->tx_chan);
		uart_dma_dump_rx_state(&dma->rx_chan);
	} else {
		printk(KERN_ERR "dma disabled, port %lu\n", idx);
		return count;
	}
	return count;
}
static ssize_t dummy_show(struct device *dev, struct device_attribute *attr, char *buf) {
	return sprintf(buf, "Please find the result in kernel log.\n");
}

static DEVICE_ATTR(uart_dma_dump, S_IRUGO | S_IWUSR, dummy_show, uart_dma_dump_states);

void uart_dma_sysfs_creat(struct device *dev) {
	device_create_file(dev, &dev_attr_uart_dma_dump);
}

#else
#define uart_dma_dump_tx_state(...)
#define uart_dma_dump_rx_state(...)
#endif

int uart_dma_tx_start(struct uart_dma_tx_chan *chan) {
	struct uart_dma_data *dma = chan->dma;
	dma->dma_base->tx_ctrl |= UART_TX_DMA_UNDER_THRES_IEN
								| UART_TX_DMA_FINISH_IEN;
	dma->dma_base->tx_ctrl &= ~UART_TX_DMA_STOP;
	dma->dma_miscbase->uart_irqs |= (UART_TX_DMA_FINISH << (8*dma->port->line));
	//dump_stack();
#ifdef DUMP_AFTER_START
	uart_dma_dump_tx_state(chan);
#endif
	return 0;
}

int uart_dma_tx_pause(struct uart_dma_tx_chan *chan, bool pause) {
	struct uart_dma_data *dma = chan->dma;
	
	//DBG_URDMA("pause %d", pause);
	if(pause)
		dma->dma_base->tx_ctrl |= UART_TX_DMA_PAUSE;
	else
		dma->dma_base->tx_ctrl &= ~UART_TX_DMA_PAUSE;
	return 0;
}

int uart_dma_tx_stop(struct uart_dma_tx_chan *chan) {
	struct uart_dma_data *dma = chan->dma;
	INFO_URDMA("tx_remain 0x%x", dma->dma_base->tx_remain);
#ifdef DUMP_BEFORE_STOP
	uart_dma_dump_tx_state(chan);
#endif
	dma->dma_base->tx_ctrl &= ~UART_TX_DMA_UNDER_THRES_IEN;
	//uart_dma_dump_tx_state(chan);
	return 0;
}

// if finish irq generated but not cleared, tx dma will really stop
int uart_dma_tx_end(struct uart_dma_tx_chan *chan) {
	struct uart_dma_data *dma = chan->dma;

	if(!chan->is_empty(chan))
	       WARN_URDMA("*** ring buffer or fifo not empty");
	dma->dma_base->tx_ctrl &= ~(UART_TX_DMA_UNDER_THRES_IEN 
	                            | UART_TX_DMA_FINISH_IEN);
#if 1
	dma->dma_base->tx_ctrl |= UART_TX_DMA_STOP;
#endif
	//uart_dma_dump_tx_state(chan);
	return 0;
}


int uart_dma_tx_clear(struct uart_dma_tx_chan *chan) {
	struct uart_dma_data *dma = chan->dma;
	
	DBG_URDMA("");
	dma->dma_base->tx_remain = 0;
	return 0;
}

int uart_dma_tx_is_empty(struct uart_dma_tx_chan *chan) {
	int empty;
	empty = !!((chan->dma->dma_base->tx_ctrl & UART_TX_DMA_FIFO_EMPTY)
			&& (chan->dma->dma_base->tx_remain == 0));
	
	DBG_URDMA("empty %d", empty);
	return empty;
}

u32 uart_dma_tx_chars(struct uart_dma_tx_chan *chan, u8 *buf, u32 size) {
	u32 count = 0;
	u32 space = 0;
	struct circ_buf *circ = &chan->circ_buf;

#if 0
	int i=0;
	INFO_URDMA("buf %p %u", buf, size);
	for(i=0; i<size; ++i)
		printk("0x%x(%c) ", buf[i], buf[i]);
#endif
	circ->tail = chan->dma->dma_base->tx_hw_rd_addr - chan->dma_base_addr;
	
	space  = CIRC_SPACE(circ->head, circ->tail, chan->buf_len);
	if(space<size) {
		size = space;
		WARN_URDMA("***, tx size too large\n");
	}
	
	count = CIRC_SPACE_TO_END(circ->head, circ->tail, chan->buf_len);
	count = min(count, size);
	if(count) { 
		memcpy(circ->buf + circ->head, buf, count);
		circ->head = (circ->head+count) & (chan->buf_len-1);
		buf += count;
		size -= count;
	}
	if(size > 0) {
		memcpy(circ->buf, buf, size);
		circ->head = (circ->head+size) & (chan->buf_len-1);
	}
	wmb();

	chan->dma->dma_base->tx_sw_wr_cnt = size+count;
	//printk("tx cnt %d\n", size+count);
	return size+count;
}

u32 uart_dma_tx_circ_buf(struct uart_dma_tx_chan *chan, struct circ_buf *xmit) {
	struct uart_dma_data *dma = chan->dma;
	u32 count = 0;
	u32 size = 0;
	
	DBG_URDMA("circ(%p %d %d)", xmit->buf, xmit->head, xmit->tail);

	size = min((u32)uart_circ_chars_pending(xmit),
					chan->buf_len - 1 - dma->dma_base->tx_remain);
	count = CIRC_CNT_TO_END(xmit->head, xmit->tail, UART_XMIT_SIZE);
	if(count > size)
		count = size;
	chan->tx_chars(&dma->tx_chan, xmit->buf+xmit->tail, count);
	if(count < size)
		chan->tx_chars(&dma->tx_chan, xmit->buf, size-count);
	xmit->tail = (xmit->tail + size) & (UART_XMIT_SIZE - 1);
	return size;
}

int uart_dma_tx_config(struct uart_dma_tx_chan *chan, u32 buf_len) {
	struct uart_dma_data *dma = chan->dma;
	
	DBG_URDMA("line %d", dma->port->line);

	if(chan->dma_base_addr) {
		ERR_URDMA("already configed!");
		return -EINVAL;
	}
	if(!dma_set_mask(chan->dma->dev, dma->dev->coherent_dma_mask)) {  //TODO: dma mask
		ERR_URDMA("dma set mask failed!");
		return -EINVAL;
	}
	chan->buf_base = dma_alloc_coherent(chan->dma->dev, buf_len, 
								&chan->dma_base_addr, GFP_KERNEL);
	if(!chan->buf_base) {
		ERR_URDMA("dma alloc failed!");
		return -ENOMEM;
	}

	dma->dma_miscbase->clk_ctrl &= ~(0x101<<(dma->port->line));

	chan->buf_len = buf_len;
	dma->dma_base->tx_buf_addr = chan->dma_base_addr;
	dma->dma_base->tx_buf_len = chan->buf_len;
	dma->dma_base->tx_buf_thres = max(16U, chan->buf_len>>7);
	dma->dma_base->tx_hw_rd_addr = chan->dma_base_addr;
	chan->circ_buf.buf = chan->buf_base;
	chan->circ_buf.tail = chan->circ_buf.head = 0;

	DBG_URDMA("vbuf 0x%p, dma addr 0x%x", chan->buf_base, chan->dma_base_addr);
	uart_dma_dump_tx_state(chan);

	return 0;
}

int uart_dma_tx_deconfig(struct uart_dma_tx_chan *chan) {
	struct uart_dma_data *dma = chan->dma;
	
	DBG_URDMA("");

	local_irq_disable();
	chan->stop(chan);
	local_irq_enable();

	if(!chan->dma_base_addr) {
		ERR_URDMA("not configed!");
		return -EINVAL;
	}

	dma_free_coherent(chan->dma->dev, chan->buf_len,
									chan->buf_base, chan->dma_base_addr);
	chan->dma_base_addr = 0;
	chan->buf_base = 0;
	chan->buf_len = 0;
	dma->dma_base->tx_buf_addr = -1;
	dma->dma_base->tx_buf_len = 0;
	chan->circ_buf.buf = 0;
	chan->circ_buf.tail = chan->circ_buf.head = 0;

	chan->dma->dma_miscbase->clk_ctrl |= (1<<(8+chan->dma->port->line));
	
	return 0;
}

void uart_dma_tx_init(struct uart_dma_tx_chan *chan) {
	DBG_URDMA("");

	chan->dma = container_of(chan, struct uart_dma_data, tx_chan);
	chan->config = uart_dma_tx_config;
	chan->deconfig = uart_dma_tx_deconfig;
	chan->start = uart_dma_tx_start;
	chan->pause = uart_dma_tx_pause;
	chan->stop = uart_dma_tx_stop;
	chan->end = uart_dma_tx_end;
	chan->clear = uart_dma_tx_clear;
	chan->is_empty= uart_dma_tx_is_empty;
	chan->tx_chars= uart_dma_tx_chars;
	chan->tx_circ_buf= uart_dma_tx_circ_buf;
}

void uart_dma_rx_enable(struct uart_dma_data *dma, int idx, bool enable) {
	if(enable)
		dma->dma_base->rx_ctrl |= UART_RX_DMA_ENABLE;
	else {
		dma->dma_base->rx_ctrl &= ~UART_RX_DMA_ENABLE;
		//dma->dma_base->rx_ctrl |= UART_RX_DMA_FINISH_IEN;
	}
}

int uart_dma_rx_start(struct uart_dma_rx_chan *chan) {
	struct uart_dma_data *dma = chan->dma;
	ERR_URDMA("");
	dma->dma_base->rx_ctrl |= UART_RX_DMA_OVER_THRES_1_IEN
								| UART_RX_DMA_OVER_THRES_2_IEN
								| UART_RX_DMA_GOING_OVERFLOW_IEN
								| UART_RX_DMA_TIMEOUT_IEN;
							    //| UART_RX_DMA_FINISH_IEN
	// rx dma will never finish until it is disabled
	
	//dump_stack();
#ifdef DUMP_AFTER_START
	uart_dma_dump_rx_state(chan);
#endif
	return 0;
}

int uart_dma_rx_stop(struct uart_dma_rx_chan *chan) {
	struct uart_dma_data *dma = chan->dma;
	ERR_URDMA("");
#ifdef DUMP_BEFORE_STOP
	uart_dma_dump_rx_state(chan);
#endif

	dma->dma_base->rx_ctrl &= ~(UART_RX_DMA_OVER_THRES_1_IEN 
								| UART_RX_DMA_OVER_THRES_2_IEN 
								| UART_RX_DMA_GOING_OVERFLOW_IEN
								| UART_RX_DMA_FINISH_IEN
								| UART_RX_DMA_TIMEOUT_IEN
								);
#if 0
#endif
	//dma->dma_base->rx_ctrl ;
	return 0;
}

int uart_dma_rx_clear(struct uart_dma_rx_chan *chan) {
	return 0;
}

bool uart_dma_rx_is_empty(struct uart_dma_rx_chan *chan) {
	struct circ_buf *circ = &chan->circ_buf;
	volatile struct uart_dma_regs __iomem *base = chan->dma->dma_base;
	bool empty;
	unsigned long flags;
	spin_lock_irqsave(&chan->lock, flags);
	circ->head = base->rx_hw_wr_addr - chan->dma_base_addr;
	empty = !CIRC_CNT(circ->head, circ->tail, chan->buf_len);
	spin_unlock_irqrestore(&chan->lock, flags);
	return empty;
}

bool uart_dma_rx_is_fifo_empty(struct uart_dma_rx_chan *chan) {
	volatile struct uart_dma_regs __iomem *base = chan->dma->dma_base;
	u32 rx_timeout = base->rx_timeout;
	base->rx_timeout = 0; // flush fifo
	wmb();
	base->rx_timeout = rx_timeout;
	return !!(base->rx_ctrl & UART_RX_DMA_FIFO_EMPTY);
}

static u32 append_buf_to_circ(struct circ_buf *circ, u32 circ_size, u8 *buf, u32 size) {
	u32 count = 0;
	u32 space = 0;

	space  = CIRC_SPACE(circ->head, circ->tail, circ_size);
	if(space<size) {
		size = space;
		WARN_URDMA("***, tx size too large\n");
	}
	
	count = CIRC_SPACE_TO_END(circ->head, circ->tail, circ_size);
	count = min(count, size);
	if(count) { 
		memcpy(circ->buf + circ->head, buf, count);
		circ->head = (circ->head+count) & (circ_size-1);
		buf += count;
		size -= count;
	}
	if(size > 0) {
		memcpy(circ->buf, buf, size);
		circ->head = (circ->head+size) & (circ_size-1);
	}
	wmb();

	return size+count;
}

static u32 append_circ_to_circ(struct circ_buf *dest, struct circ_buf *src, u32 circ_size) {
	u32 count;
	u32 trans;
	count = CIRC_CNT(src->head, src->tail, circ_size);
	trans = CIRC_SPACE(dest->head, dest->tail, circ_size);
	trans = min(trans, count);
	
	count = CIRC_CNT_TO_END(src->head, src->tail, circ_size);
	count = min(trans, count);
	if(count) {
		append_buf_to_circ(dest, circ_size, src->buf + src->tail, count);
		src->tail = (src->tail + count) & (circ_size - 1);
	}
	count = trans - count;
	if(count) {
		if(src->tail != 0)
			ERR_URDMA("src->tail != 0. FATAL");
		append_buf_to_circ(dest, circ_size, src->buf + src->tail, count);
		src->tail = (src->tail + count) & (circ_size - 1);
	}

	return trans;
}

int uart_dma_rx_circ_buf(struct uart_dma_rx_chan *chan) {
	struct circ_buf *circ = &chan->circ_buf;
	struct circ_buf *shadow = &chan->shadow_buf;
	volatile struct uart_dma_regs __iomem *base = chan->dma->dma_base;
	u32 count = 0;
	rmb();

	//spin_lock(&chan->lock);
	circ->head = base->rx_hw_wr_addr - chan->dma_base_addr;
	DBG_URDMA("cnt in dma/remain 0x%x/0x%x", 
				CIRC_CNT(circ->head, circ->tail, chan->buf_len),
				base->rx_remain);
	count = append_circ_to_circ(shadow, circ, chan->buf_len);
	base->rx_sw_rd_cnt = count;
	//wmb();
	
	DBG_URDMA("cnt in dma/remain/shadow 0x%x/0x%x/0x%x", 
				CIRC_CNT(circ->head, circ->tail, chan->buf_len),
				base->rx_remain,
				CIRC_CNT(shadow->head, shadow->tail, chan->buf_len));
#ifdef DUMP_RECV_CHARS
do {
	struct timespec ts;
	unsigned long long tt;
	getnstimeofday(&ts);
	tt = (unsigned long long)ts.tv_sec*1000000+ts.tv_nsec/1000;

	printk("%llu us cnt %d ", tt, count);
	for( ; count; count --) {
		char ch = shadow->buf[shadow->tail];
		shadow->tail = (shadow->tail + 1) & (chan->buf_len - 1);
		printk("0x%x(%c)", ch, ch);	
	}
	printk("\n");
} while(0);
#endif
	//spin_unlock(&chan->lock);

	return count;
}

int uart_dma_rx_config(struct uart_dma_rx_chan *chan, u32 buf_len) {
	struct uart_dma_data *dma = chan->dma;
	volatile struct uart_dma_regs __iomem *base = dma->dma_base;
	u32 rx_timeout;

	DBG_URDMA("line %d", dma->port->line);

	if(chan->dma_base_addr) {
		ERR_URDMA("already configed!");
		return -EINVAL;
	}
	if(!dma_set_mask(dma->dev, DMA_BIT_MASK(32))) {
		ERR_URDMA("dma set mask failed!");
		return -EINVAL;
	}
	if(buf_len <= 16U) {
		WARN_URDMA("ring buf_len %d is too small, use 128 instead", buf_len);
		buf_len = 128U;
	}
	chan->buf_base = dma_alloc_coherent(dma->dev, buf_len, 
								&chan->dma_base_addr, GFP_KERNEL);
	if(!chan->buf_base) {
		ERR_URDMA("dma alloc failed!");
		return -ENOMEM;
	}

	chan->shadow_buf.buf = kmalloc(buf_len, GFP_KERNEL);
	if(!chan->shadow_buf.buf) {
		ERR_URDMA("shadow buf alloc failed!");
		dma_free_coherent(chan->dma->dev, chan->buf_len,
										chan->buf_base, chan->dma_base_addr);
		return -ENOMEM;
	}


	dma->dma_miscbase->clk_ctrl &= ~(0x101<<(dma->port->line));

	chan->buf_len = buf_len;
	base->rx_buf_addr = chan->dma_base_addr;
	base->rx_buf_len = chan->buf_len;
	base->rx_buf_thres_1 = base->rx_buf_thres_2 = min(chan->buf_len - 16U, chan->buf_len - (chan->buf_len>>4));
	base->rx_hw_wr_addr = chan->dma_base_addr;
	chan->circ_buf.buf = chan->buf_base;
	chan->circ_buf.tail = chan->circ_buf.head = 0;
	chan->shadow_buf.tail = chan->shadow_buf.head = 0;

	if(!dma->baud) dma->baud = 115200;
	rx_timeout = 10*26000000UL/dma->baud*32; // 32 Bytes time, 26M is uart dma clock
	if(rx_timeout >= 0xFFFF) {
		rx_timeout = 0xFFFE;
	}
	base->rx_timeout = rx_timeout;
	INFO_URDMA("rx_timeout %u", base->rx_timeout);
	
	uart_dma_rx_enable(dma, dma->port->line, 1);

	DBG_URDMA("vbuf 0x%p, dma addr 0x%x", chan->buf_base, chan->dma_base_addr);
	uart_dma_dump_rx_state(chan);

	return 0;	
}

int uart_dma_rx_deconfig(struct uart_dma_rx_chan *chan) {
	struct uart_dma_data *dma = chan->dma;
	volatile struct uart_dma_regs __iomem *base = dma->dma_base;

	DBG_URDMA("");

	local_irq_disable();
	uart_dma_rx_enable(dma, dma->port->line, 0);
	chan->stop(chan);
	local_irq_enable();

	if(!chan->dma_base_addr) {
		ERR_URDMA("not configed!");
		return -EINVAL;
	}


	if(!chan->shadow_buf.buf) {
		ERR_URDMA("shadow buf null! why?");
		//return -EINVAL;
	} else {
		kfree(chan->shadow_buf.buf);
		chan->shadow_buf.buf = NULL;
	}

	dma_free_coherent(dma->dev, chan->buf_len, 
								chan->buf_base, chan->dma_base_addr);
	chan->dma_base_addr = 0;
	chan->buf_base = 0;
	
	chan->shadow_buf.tail = chan->shadow_buf.head = 0;
	chan->circ_buf.buf = chan->buf_base;
	chan->circ_buf.tail = chan->circ_buf.head = 0;
	
	chan->buf_len = 0;
	base->rx_buf_len = 0;
	base->rx_buf_addr = -1;
	base->rx_buf_thres_1 = 0;
	base->rx_buf_thres_2 = 0;
	base->rx_hw_wr_addr = -1;
	
	dma->dma_miscbase->clk_ctrl |= (1<<(8+dma->port->line));

	DBG_URDMA("vbuf 0x%p, dma addr 0x%x", chan->buf_base, chan->dma_base_addr);

	return 0;	
}

void uart_dma_rx_init(struct uart_dma_rx_chan *chan) {
	spin_lock_init(&chan->lock);
	chan->dma = container_of(chan, struct uart_dma_data, rx_chan);
	chan->config = uart_dma_rx_config;
	chan->deconfig = uart_dma_rx_deconfig;
	chan->start = uart_dma_rx_start;
	chan->stop = uart_dma_rx_stop;
	chan->clear = uart_dma_rx_clear;
	chan->is_empty= uart_dma_rx_is_empty;
	chan->rx_circ_buf= uart_dma_rx_circ_buf;
}

#if 0
#endif
