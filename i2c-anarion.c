/*
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/delay.h>

#define I2C_CKDIV_REG 			0x0
#define I2C_CKDIV_DIVISOR		(0x3FF << 0

#define I2C_INTCTL_REG 			0x4
#define I2C_INTCTL_CMD_THR		(0xF << 0)
#define I2C_INTCTL_CMD_THR_SHIFT	(0x0)
#define I2C_INTCTL_CMD_EPTY_ITR 	(0x1 << 4)
#define I2C_INTCTL_CMD_UDR_THR_ITR  	(0x1 << 5)
#define I2C_INTCTL_RD_THR		(0xF << 8)
#define I2C_INTCTL_RD_THR_SHIFT		(0x8)
#define I2C_INTCTL_RD_FULL_ITR		(0x1 << 12)
#define I2C_INTCTL_RD_OVR_THR_ITR	(0x1 << 13)
#define I2C_INTCTL_NO_ADR_ACK_ITR	(0x1 << 16)
#define I2C_INTCTL_NO_DATA_ACK_ITR	(0x1 << 17)
#define I2C_INTCTL_ITR			(I2C_INTCTL_CMD_EPTY_ITR|I2C_INTCTL_CMD_UDR_THR_ITR|\
				I2C_INTCTL_RD_FULL_ITR|I2C_INTCTL_RD_OVR_THR_ITR|\
				I2C_INTCTL_NO_ADR_ACK_ITR|I2C_INTCTL_NO_DATA_ACK_ITR)

#define I2C_STA_REG			(0x8)
#define I2C_STA_CMD_FIFO		(0x1F << 0)
#define I2C_STA_CMD_FIFO_FULL		(0x10 << 0)
#define I2C_STA_DATA_FIFO		(0x1F << 8)
#define I2C_STA_DATA_FIFO_SHIFT		(0x8)
#define I2C_STA_NO_ADR_ACK		(0x1 << 16)
#define I2C_STA_NO_DATA_ACK		(0x1 << 17)
#define I2C_STA_CMD_EPTY		(0x1 << 18)
#define I2C_STA_CMD_UDR_THR		(0x1 << 19)
#define I2C_STA_RD_FULL			(0x1 << 20)
#define I2C_STA_RD_OVR_THR		(0x1 << 21)
#define I2C_STA_STATE			(0x3 << 30)

#define I2C_CQCTL_REG			(0x0C)
#define I2C_CQCTL_RESUME        (0x0 << 0)
#define I2C_CQCTL_PAUSE			(0x1 << 0)
#define I2C_CQCTL_FLUSH			(0x1 << 1)

#define I2C_SADR_REG			(0x10)
#define I2C_SADR_SLAVE_ADDR		(0x7F << 0)

#define I2C_RXFIFO_REG			(0x14)
#define I2C_RXFIFO_DATA			(0xF << 0)

#define I2C_WTCMD_REG			(0x20)
#define I2C_WTCMD_VAL			(0xF << 0)

#define I2C_RDCMD_REG			(0x24)
#define I2C_RDCMD_VAL			(0xF << 0)

#define I2C_LTWTCMD_REG			(0x28)
#define I2C_LTWTCMD_VAL			(0xF << 0)

#define I2C_LTRDCMD_REG			(0x2C)
#define I2C_LTRDCMD_VAL			(0xF << 0)

#define I2C_MANSTT_REG			(0x30)
#define I2C_MANSTT_CMD			(0xF << 0)

#define I2C_MANSTP_REG			(0x34)
#define I2C_MANSTP_CMD			(0xF << 0)

#define ANARION_I2C_TIMEOUT (msecs_to_jiffies(1000))
#define ANARION_I2C_CDIV_MAX (0x3ff)

#define ANARION_REPEAT_START (0x1 << 0)
struct anarion_i2c_dev {
	struct device *dev;
	void __iomem *regs;
	struct clk *clk;
	int irq;
	struct i2c_adapter adapter;
	struct completion completion;
	struct i2c_msg *curr_msg;
	u32 msg_err;
	u8 *msg_buf;
	size_t msg_buf_remaining;
	size_t msg_queue_remaining;
	int flag;
};

static inline void anarion_i2c_writel(struct anarion_i2c_dev *i2c_dev,
				      u32 reg, u32 val)
{
	//printk(KERN_INFO "i2c: write %x value %x", i2c_dev->regs + reg, val);
	writel(val, i2c_dev->regs + reg);
}

static inline u32 anarion_i2c_readl(struct anarion_i2c_dev *i2c_dev, u32 reg)
{
	return readl(i2c_dev->regs + reg);
}

static inline void anarion_enable_interrupt(struct anarion_i2c_dev *i2c_dev, u32 irq)
{
	u32 int_ctl;
	int_ctl = anarion_i2c_readl(i2c_dev, I2C_INTCTL_REG);
	int_ctl |= irq;
	anarion_i2c_writel(i2c_dev, I2C_INTCTL_REG, int_ctl);
}

static inline int anarion_check_interrupt(struct anarion_i2c_dev *i2c_dev, u32 irq)
{
	u32 int_ctl;
	int_ctl = anarion_i2c_readl(i2c_dev, I2C_INTCTL_REG);
	return int_ctl & irq;
}

static inline void anarion_disable_interrupt(struct anarion_i2c_dev *i2c_dev, u32 irq)
{
	u32 int_ctl;
	int_ctl = anarion_i2c_readl(i2c_dev, I2C_INTCTL_REG);
	int_ctl &= ~irq;
	anarion_i2c_writel(i2c_dev, I2C_INTCTL_REG, int_ctl);
}
static void anarion_fill_comb_txfifo(struct anarion_i2c_dev *i2c_dev)
{
	u32 val;
	while (i2c_dev->msg_buf_remaining) {
		val = anarion_i2c_readl(i2c_dev, I2C_STA_REG);
		if ((val & I2C_STA_CMD_FIFO_FULL)){
			break;
		}
		anarion_i2c_writel(i2c_dev, I2C_WTCMD_REG,
				   *i2c_dev->msg_buf);
		i2c_dev->msg_buf++;
		i2c_dev->msg_buf_remaining--;
	}
}

static void anarion_fill_txfifo(struct anarion_i2c_dev *i2c_dev)
{
	u32 val;
    anarion_i2c_writel(i2c_dev, I2C_CQCTL_REG, I2C_CQCTL_PAUSE);
	while (i2c_dev->msg_buf_remaining) {
		val = anarion_i2c_readl(i2c_dev, I2C_STA_REG);
		if ((val & I2C_STA_CMD_FIFO_FULL)){
			break;
		}
		if (i2c_dev->msg_buf_remaining == 1  && !(i2c_dev->flag & ANARION_REPEAT_START)) { // last one
			anarion_i2c_writel(i2c_dev, I2C_LTWTCMD_REG, 
				   *i2c_dev->msg_buf);
		}
		else {
			anarion_i2c_writel(i2c_dev, I2C_WTCMD_REG,
				   *i2c_dev->msg_buf);
		}
		i2c_dev->msg_buf++;
		i2c_dev->msg_buf_remaining--;
	}
    anarion_i2c_writel(i2c_dev, I2C_CQCTL_REG, I2C_CQCTL_RESUME);
	// if buffer is empty, wait for command queue empty ITR.
	// if not, enable comand queue under threshold ITR.
	if (i2c_dev->msg_buf_remaining){
		anarion_enable_interrupt(i2c_dev, I2C_INTCTL_CMD_UDR_THR_ITR);
	}
	else{
		anarion_enable_interrupt(i2c_dev, I2C_INTCTL_CMD_EPTY_ITR);
	}
}

static void anarion_drain_rxfifo(struct anarion_i2c_dev *i2c_dev)
{
	u32 val;
	int i;
	while (i2c_dev->msg_buf_remaining) {
		val = anarion_i2c_readl(i2c_dev, I2C_STA_REG);
		val = (val >> 8) & 0x1f;
		if (!val){
			break;
		}
		*i2c_dev->msg_buf = anarion_i2c_readl(i2c_dev,
						      I2C_RXFIFO_REG) & 0xff;
		i2c_dev->msg_buf++;
		i2c_dev->msg_buf_remaining--;
	}

	if (i2c_dev->msg_buf_remaining == 0) {
		return;
	}
	anarion_i2c_writel(i2c_dev, I2C_CQCTL_REG, I2C_CQCTL_PAUSE);
	// notify controller to read more data
	while (i2c_dev->msg_queue_remaining) {
		val = anarion_i2c_readl(i2c_dev, I2C_STA_REG);
		if ((val & I2C_STA_CMD_FIFO_FULL)){
			break;
		}
		if ((i2c_dev->msg_queue_remaining==1) && !(i2c_dev->flag & ANARION_REPEAT_START))
			anarion_i2c_writel(i2c_dev, I2C_LTRDCMD_REG, 0x00);
		else
			anarion_i2c_writel(i2c_dev, I2C_RDCMD_REG, 0x00);
		i2c_dev->msg_queue_remaining--;
	}
	anarion_i2c_writel(i2c_dev, I2C_CQCTL_REG, I2C_CQCTL_RESUME);
	/* if cmd queue is full, and there are still data to read, enable another transfer */
	if (i2c_dev->msg_queue_remaining){
		anarion_enable_interrupt(i2c_dev, I2C_INTCTL_RD_OVR_THR_ITR);
	}
	else{
		anarion_enable_interrupt(i2c_dev, I2C_INTCTL_CMD_EPTY_ITR);
	}
}

static void anarion_drain_rxfifo_isr(struct anarion_i2c_dev *i2c_dev)
{
	u32 sta_reg, int_reg;
	int i;
	u32 val;
	u32 depth;
	sta_reg  = anarion_i2c_readl(i2c_dev, I2C_STA_REG);
	int_reg = anarion_i2c_readl(i2c_dev, I2C_INTCTL_REG);
	depth = (sta_reg >> 8) & 0x1f;
	while (i2c_dev->msg_buf_remaining && depth) {
		*i2c_dev->msg_buf = anarion_i2c_readl(i2c_dev,
						      I2C_RXFIFO_REG);
		i2c_dev->msg_buf++;
		i2c_dev->msg_buf_remaining--;
		depth --;
	}
	if (i2c_dev->msg_buf_remaining == 0) {
		return;
	}
	anarion_i2c_writel(i2c_dev, I2C_CQCTL_REG, I2C_CQCTL_PAUSE);
	// check the number of data fifo available, then write to command Q.
	depth = (sta_reg >> 8) & 0x1f;
	while (i2c_dev->msg_queue_remaining && depth) {
		val = anarion_i2c_readl(i2c_dev, I2C_STA_REG);
		if ((val & I2C_STA_CMD_FIFO_FULL))
			break;
		if ((i2c_dev->msg_queue_remaining==1) && !(i2c_dev->flag & ANARION_REPEAT_START)) {
			anarion_i2c_writel(i2c_dev, I2C_LTRDCMD_REG, 0x00);
		}
		else {
			anarion_i2c_writel(i2c_dev, I2C_RDCMD_REG, 0x00);
		}
		i2c_dev->msg_queue_remaining--;
		depth--;
	}
	anarion_i2c_writel(i2c_dev, I2C_CQCTL_REG, I2C_CQCTL_RESUME);
	/* if cmd queue is full, and there are still data to read, enable another transfer */
	if (i2c_dev->msg_queue_remaining)
		anarion_enable_interrupt(i2c_dev, I2C_INTCTL_RD_OVR_THR_ITR);
	else
		anarion_enable_interrupt(i2c_dev, I2C_INTCTL_CMD_EPTY_ITR);
}

static irqreturn_t anarion_i2c_isr(int this_irq, void *data)
{
	struct anarion_i2c_dev *i2c_dev = data;
	u32 val, err;
	i2c_dev->msg_err = 0;

	val = anarion_i2c_readl(i2c_dev, I2C_STA_REG);
	err = val & (I2C_STA_NO_ADR_ACK| I2C_STA_NO_DATA_ACK);
	if (err) {
		i2c_dev->msg_err = err;
		anarion_i2c_writel(i2c_dev, I2C_STA_REG, val);
		anarion_disable_interrupt(i2c_dev, I2C_INTCTL_ITR);
		complete(&i2c_dev->completion);
		return IRQ_HANDLED;
	}

	if (anarion_check_interrupt(i2c_dev,I2C_INTCTL_CMD_EPTY_ITR) && (val & I2C_STA_CMD_EPTY)) {
		anarion_disable_interrupt(i2c_dev, I2C_INTCTL_CMD_EPTY_ITR);
		complete(&i2c_dev->completion);
		return IRQ_HANDLED;
	}
	if (anarion_check_interrupt(i2c_dev,I2C_INTCTL_CMD_UDR_THR_ITR ) && (val & I2C_STA_CMD_UDR_THR)) {
		anarion_disable_interrupt(i2c_dev, I2C_INTCTL_CMD_UDR_THR_ITR);
		anarion_fill_txfifo(i2c_dev);
		return IRQ_HANDLED;
	}

	if (anarion_check_interrupt(i2c_dev, I2C_INTCTL_RD_OVR_THR_ITR) && (val & I2C_STA_RD_OVR_THR)) {
		anarion_disable_interrupt(i2c_dev, I2C_INTCTL_RD_OVR_THR_ITR);
		anarion_drain_rxfifo_isr(i2c_dev);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}
static void anarion_i2c_init(struct anarion_i2c_dev *i2c_dev, struct i2c_msg *msg)
{
	u32 val;
	anarion_i2c_writel(i2c_dev, I2C_SADR_REG, msg->addr);	
	anarion_enable_interrupt(i2c_dev, I2C_INTCTL_NO_ADR_ACK_ITR|
					I2C_INTCTL_NO_DATA_ACK_ITR );
	/* set read fifo interrupt threshold */
	val = anarion_i2c_readl(i2c_dev, I2C_INTCTL_REG );
	val &= ~I2C_INTCTL_RD_THR;
	val |= (0x4<< I2C_INTCTL_RD_THR_SHIFT );
	anarion_i2c_writel(i2c_dev, I2C_INTCTL_REG, val);
	/* set write fifo interrupt threshold */
	val = anarion_i2c_readl(i2c_dev, I2C_INTCTL_REG );
	// clear status
	anarion_i2c_writel(i2c_dev, I2C_STA_REG, ((0x01 << 16) | (0x01 << 17)));
	// flush command queue entries
	anarion_i2c_writel(i2c_dev, I2C_CQCTL_REG, I2C_CQCTL_FLUSH);
	// set device address
	anarion_i2c_writel(i2c_dev, I2C_SADR_REG, msg->addr);
	// read out rx fifo
	val = anarion_i2c_readl(i2c_dev, I2C_STA_REG) & I2C_STA_DATA_FIFO;
	while (val ) {
		anarion_i2c_readl(i2c_dev, I2C_RXFIFO_REG);
		val = anarion_i2c_readl(i2c_dev, I2C_STA_REG) & I2C_STA_DATA_FIFO;
	}
	anarion_disable_interrupt(i2c_dev, I2C_INTCTL_CMD_UDR_THR_ITR|I2C_INTCTL_CMD_EPTY_ITR);
}

static int anarion_i2c_comb_xfer_msg(struct anarion_i2c_dev *i2c_dev,
				struct i2c_msg *msg)
{
	u32 val;
	unsigned long time_left;
	u32 done =0;
	u32 comb_len = msg[0].len + msg[1].len ;

	anarion_i2c_init(i2c_dev, msg);

	i2c_dev->curr_msg = &msg[0];
	i2c_dev->msg_buf = msg[0].buf;
	i2c_dev->msg_buf_remaining = msg[0].len;

	anarion_i2c_writel(i2c_dev, I2C_CQCTL_REG, I2C_CQCTL_PAUSE);

	// write to tx fifo  without enabling interrupt
	anarion_fill_comb_txfifo(i2c_dev);
	
	i2c_dev->curr_msg = &msg[1];
	i2c_dev->msg_buf = msg[1].buf;
	i2c_dev->msg_buf_remaining = msg[1].len;
	i2c_dev->msg_queue_remaining = msg[1].len;

	anarion_drain_rxfifo(i2c_dev);

	// unpause 
	anarion_i2c_writel(i2c_dev, I2C_CQCTL_REG, I2C_CQCTL_RESUME);
	time_left = wait_for_completion_timeout(&i2c_dev->completion,
						ANARION_I2C_TIMEOUT);
	reinit_completion(&i2c_dev->completion);
	if (!time_left) {
		dev_err(i2c_dev->dev, "i2c transfer timed out\n");
		return -ETIMEDOUT;
	}
	anarion_drain_rxfifo(i2c_dev);

	if (likely(!i2c_dev->msg_err)) {
		return 0;
	}

	dev_err(i2c_dev->dev, "i2c transfer failed: %x\n", i2c_dev->msg_err);

	if (i2c_dev->msg_err & (I2C_STA_NO_ADR_ACK|I2C_STA_NO_ADR_ACK))
		return -EREMOTEIO;
	else
		return -EIO;
}

static int anarion_i2c_xfer_msg(struct anarion_i2c_dev *i2c_dev,
				struct i2c_msg *msg)
{
	u32 val;
	unsigned long time_left;
	u32 done =0;
	anarion_i2c_init(i2c_dev, msg);

	i2c_dev->curr_msg = msg;
	i2c_dev->msg_buf = msg->buf;
	i2c_dev->msg_buf_remaining = msg->len;
	i2c_dev->msg_queue_remaining = msg->len;
    
	if (msg->flags & I2C_M_RD) {
		anarion_drain_rxfifo(i2c_dev);
	} else {
		anarion_fill_txfifo(i2c_dev);
	}

	time_left = wait_for_completion_timeout(&i2c_dev->completion,
						ANARION_I2C_TIMEOUT);

	if (!time_left) {
		dev_err(i2c_dev->dev, "i2c transfer timed out\n");
		return -ETIMEDOUT;
	}
	if (msg->flags & I2C_M_RD) {
		anarion_drain_rxfifo(i2c_dev);
	}

	if (likely(!i2c_dev->msg_err)) {
		return 0;
	}
    printk("%s, %d\n", __func__, __LINE__);
	dev_err(i2c_dev->dev, "i2c transfer failed: %x\n", i2c_dev->msg_err);

	if (i2c_dev->msg_err & (I2C_STA_NO_ADR_ACK|I2C_STA_NO_ADR_ACK))
		return -EREMOTEIO;
	else
		return -EIO;
}

static int anarion_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[],
			    int num)
{
	struct anarion_i2c_dev *i2c_dev = i2c_get_adapdata(adap);
	int i;
	int ret = 0;

	reinit_completion(&i2c_dev->completion);
	if (num == 2) {
		if ((msgs[0].flags & I2C_M_RD) == 0 &&
		    (msgs[1].flags & I2C_M_RD) == 1) {
			ret = anarion_i2c_comb_xfer_msg(i2c_dev, &msgs[i]);
			return ret ? :2;
		}
	}

	for (i = 0; i < num; i++) {
		ret = anarion_i2c_xfer_msg(i2c_dev, &msgs[i]);
		if (ret<0)
			break;
	}

	if(i2c_dev->irq == 24)
			udelay(2000);
	
	return ret ? : i;
}

static u32 anarion_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm anarion_i2c_algo = {
	.master_xfer	= anarion_i2c_xfer,
	.functionality	= anarion_i2c_func,
};

/*
 * This HW was reported to have problems with clock stretching:
 * http://www.advamation.com/knowhow/raspberrypi/rpi-i2c-bug.html
 * https://www.raspberrypi.org/forums/viewtopic.php?p=146272
 */
static const struct i2c_adapter_quirks anarion_i2c_quirks = {
	.flags = I2C_AQ_NO_CLK_STRETCH,
};

static int anarion_i2c_probe(struct platform_device *pdev)
{
	struct anarion_i2c_dev *i2c_dev;
	struct resource *mem, *irq;
	u32 bus_clk_rate, divider;
	int ret;
	struct i2c_adapter *adap;
	printk(KERN_DEBUG "anarion i2c driver probe");

	i2c_dev = devm_kzalloc(&pdev->dev, sizeof(*i2c_dev), GFP_KERNEL);
	if (!i2c_dev)
		return -ENOMEM;
	platform_set_drvdata(pdev, i2c_dev);
	i2c_dev->dev = &pdev->dev;
	init_completion(&i2c_dev->completion);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	i2c_dev->regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(i2c_dev->regs))
		return PTR_ERR(i2c_dev->regs);

	ret = of_property_read_u32(pdev->dev.of_node, "clock-frequency",
				   &bus_clk_rate);
	if (ret < 0) {
		dev_warn(&pdev->dev,
			 "Could not read clock-frequency property\n");
		bus_clk_rate = 100000;
	}

	divider = (12000000 / (4 *(bus_clk_rate) ) ) - 1;

	if (divider > ANARION_I2C_CDIV_MAX) {
		dev_err(&pdev->dev, "Invalid clock-frequency\n");
		return -ENODEV;
	}
	anarion_i2c_writel(i2c_dev, I2C_CKDIV_REG , divider);

	irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!irq) {
		dev_err(&pdev->dev, "No IRQ resource\n");
		return -ENODEV;
	}
	i2c_dev->irq = irq->start;

	ret = request_irq(i2c_dev->irq, anarion_i2c_isr, IRQF_SHARED,
			  dev_name(&pdev->dev), i2c_dev);
	if (ret) {
		dev_err(&pdev->dev, "Could not request IRQ\n");
		return -ENODEV;
	}

	adap = &i2c_dev->adapter;
	i2c_set_adapdata(adap, i2c_dev);
	adap->owner = THIS_MODULE;
	adap->class = I2C_CLASS_DEPRECATED;
	strlcpy(adap->name, "anarion I2C adapter", sizeof(adap->name));
	adap->algo = &anarion_i2c_algo;
	adap->dev.parent = &pdev->dev;
	adap->dev.of_node = pdev->dev.of_node;
	adap->quirks = &anarion_i2c_quirks;

	ret = i2c_add_adapter(adap);
	if (ret) {
		printk(KERN_DEBUG "anarion i2c driver initialization faiure");
		free_irq(i2c_dev->irq, i2c_dev);
	}
	else
		printk(KERN_DEBUG "anarion i2c driver initialized");
	return ret;
}

static int anarion_i2c_remove(struct platform_device *pdev)
{
	struct anarion_i2c_dev *i2c_dev = platform_get_drvdata(pdev);

	free_irq(i2c_dev->irq, i2c_dev);
	i2c_del_adapter(&i2c_dev->adapter);

	return 0;
}

static const struct of_device_id anarion_i2c_of_match[] = {
	{ .compatible = "adaptrum,anarion-i2c" },
	{},
};
MODULE_DEVICE_TABLE(of, anarion_i2c_of_match);

static struct platform_driver anarion_i2c_driver = {
	.probe		= anarion_i2c_probe,
	.remove		= anarion_i2c_remove,
	.driver		= {
		.name	= "i2c-anarion",
		.of_match_table = anarion_i2c_of_match,
	},
};
module_platform_driver(anarion_i2c_driver);

MODULE_DESCRIPTION("ANARION I2C bus adapter");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:i2c-anarion");
