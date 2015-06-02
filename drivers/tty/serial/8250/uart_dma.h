#ifndef __UART_DMA__
#define __UART_DMA__


#ifdef CONFIG_IM_UART_DMA
// uartdma_base

// UR_GCLK_REG		@(uart_base+0x0000)
// UR_RXDMAEN_REG	@(uart_base+0x0004)



 struct uart_dma_misc_regs {  //0xFFEA_8004
#define UART_RX_DMA_OVER_THRES_1       1
#define UART_RX_DMA_OVER_THRES_2      (1<<1)
#define UART_RX_DMA_FINISH            (1<<2)
#define UART_RX_DMA_GOING_OVERFLOW    (1<<3)

#define UART_TX_DMA_UNDER_THRES       (1<<4)
#define UART_TX_DMA_FINISH            (1<<5)

#define UART_RX_DMA_TIMEOUT           (1<<6)   // Be careful

#define UART_RX_DMA_IRQ_MASK 0x4F
#define UART_TX_DMA_IRQ_MASK 0x30

	union {
    	u8 uart_irq[4];         //irq flags
    	u32 uart_irqs;
	};

#define UART1_DMA_CLK_GATE_OFF         1
#define UART2_DMA_CLK_GATE_OFF        (1<<1)
#define UART3_DMA_CLK_GATE_OFF        (1<<2)
	union {
		struct {
			u8 uart_clk_ctrl;
			u8 dma_clk_ctrl;
		};
		u32 clk_ctrl;          //gated clock control
	};

};

struct uart_dma_regs {   // 0xFFEA_8180 for uart1
     dma_addr_t rx_buf_addr;
     u32 rx_buf_len;
     u32 rx_sw_rd_cnt;
     dma_addr_t rx_hw_wr_addr;
     u32 rx_remain;
     u32 rx_buf_thres_1;
     u32 rx_buf_thres_2;
     dma_addr_t rx_sw_rd_addr;

#define UART_RX_DMA_FIFO_EMPTY        (1<<15)
#define UART_RX_DMA_OVER_THRES_1_IEN  (1<<16)
#define UART_RX_DMA_OVER_THRES_2_IEN  (1<<17)
#define UART_RX_DMA_FINISH_IEN        (1<<18)
#define UART_RX_DMA_GOING_OVERFLOW_IEN    (1<<19)
#define UART_RX_DMA_TIMEOUT_IEN       (1<<20)
#define UART_RX_DMA_TIMEOUT_TRICKY_IEN       (1<<22)
#define UART_RX_DMA_IEN_MASK          (0x1F<<16)   //TIMEOUT IEN is specical

#define UART_RX_DMA_ENABLE            (1<<31)
     u32 rx_ctrl;
     u32 rx_timeout;

     u8 reserverd[0xFFEA81C0UL-0xFFEA81A8UL];

     dma_addr_t tx_buf_addr;
     u32 tx_buf_len;
     u32 tx_sw_wr_cnt;
     dma_addr_t tx_hw_rd_addr;
     u32 tx_remain;
     u32 tx_buf_thres;
     dma_addr_t tx_sw_wr_addr;

#define UART_TX_DMA_PAUSE              1
#define UART_TX_DMA_STOP              (1<<2)
#define UART_TX_DMA_FIFO_EMPTY        (1<<15)
#define UART_TX_DMA_UNDER_THRES_IEN   (1<<16)
#define UART_TX_DMA_FINISH_IEN        (1<<17)
#define UART_TX_DMA_IEN_MASK          (0x3<<16)
     u32 tx_ctrl;
};

struct uart_dma_tx_chan {
	struct uart_dma_data *dma;
	dma_addr_t dma_base_addr;
	u8 *buf_base;
	u32 buf_len;

	struct circ_buf circ_buf;

	//dma_addr_t dma_hw_rd_addr;
	//u8 *next_sw_wr_addr;
	//u32 last_wr_cnt;

	int (*config)(struct uart_dma_tx_chan *chan, u32 buf_len);
	int (*deconfig)(struct uart_dma_tx_chan *chan);
	int (*start)(struct uart_dma_tx_chan *chan);
	u32 (*tx_chars)(struct uart_dma_tx_chan *chan, u8 *buf, u32 size);
	u32 (*tx_circ_buf)(struct uart_dma_tx_chan *chan, struct circ_buf *xmit);
	int (*stop)(struct uart_dma_tx_chan *chan);
	int (*end)(struct uart_dma_tx_chan *chan);
	int (*pause)(struct uart_dma_tx_chan *chan, bool pause);
	int (*clear)(struct uart_dma_tx_chan *chan);
	int (*is_empty)(struct uart_dma_tx_chan *chan);
};

void uart_dma_dump_tx_state(struct uart_dma_tx_chan *chan);
void uart_dma_tx_init(struct uart_dma_tx_chan *chan);


struct uart_dma_rx_chan {
	spinlock_t lock;
	struct uart_dma_data *dma;
	dma_addr_t dma_base_addr;
	u8 *buf_base;
	u32 buf_len;
	
	struct circ_buf circ_buf;
	struct circ_buf shadow_buf;

	u32 sw_rd_addr;
	u32 last_rd_addr;

	int (*config)(struct uart_dma_rx_chan *chan, u32 buf_len);
	int (*deconfig)(struct uart_dma_rx_chan *chan);
	int (*start)(struct uart_dma_rx_chan *chan);
	int (*rx_circ_buf)(struct uart_dma_rx_chan *chan);
	int (*stop)(struct uart_dma_rx_chan *chan);
	int (*pause)(struct uart_dma_rx_chan *chan);
	int (*clear)(struct uart_dma_rx_chan *chan);
	bool (*is_empty)(struct uart_dma_rx_chan *chan);
};

void uart_dma_rx_init(struct uart_dma_rx_chan *chan);


struct uart_dma_data {
	int enable;
	volatile struct uart_dma_misc_regs * __iomem dma_miscbase;
	volatile struct uart_dma_regs * __iomem dma_base;
	struct resource misc_res;
	struct resource res;
	unsigned long irq;
	struct device *dev;
	struct uart_port *port;
	unsigned long baud;

	struct uart_dma_tx_chan tx_chan;
	struct uart_dma_rx_chan rx_chan;
};

void uart_dma_sysfs_creat(struct device *dev);

#endif

#endif
