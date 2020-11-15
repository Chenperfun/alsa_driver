# 说明

​		该项目是基于友善之臂的mini2440单板，实现内核ALSA声卡驱动的完全移植与重构，实现了一个经典的平台驱动设备总线的模型。对ALSA声卡设备的完美支持，并将自己写的驱动放入内核，成功编译通过并执行。该驱动的框架包含如下三个部分：

​		1、machine：单板相关，通过设备私有数据的link成员，决定单板（mini2440）使用的CPU与codec两侧的dai（*Digital* *Audio* *Interfaces*(音频设备的硬件接口），以及两侧用于数据传输的mda传输通道。

​		2、platform：实现CPU侧用于ALSA声卡驱动的dai以及dma的驱动。

​		3、codec：音频解码芯片uad1341的硬件相关，包含dai控制传输以及数据传输两个通道。



# 项目框架

​		ALSA声卡的驱动框架如图所示：

​										![](C:\Users\Administrator\Desktop\ASOC声卡驱动\框架.png)

​		首先，一个ALSA声卡架构应当包含如下的逻辑设备接口：

​		**/dev/pcmC0D0c:用于录音的pcm设备**
​		**/dev/pcmC0D0p:用于播放的pcm设备**
​		/dev/timer:定时器
​		**/dev/controlC0:用于声卡的控制,如通道选择**
​		/dev/mixer:混音处理		

​		其中我们重点关注加黑字段：录音设备、播放设备、控制接口三个部分。

​		CPU侧与codec通过两条通道进行数据传输，分别用于<u>属性设置</u>与<u>数据传输</u>。具体来说在本驱动中，CPU通过DAI和codec进行属性设置，在mini2440中，使用的是IIS协议。属性传输由DMA通道2进行传输。



# ALSA简介

​		linux下的声卡驱动架构主要分为**OSS架构**和**ALSA架构**。OSS全称是Open Sound System，叫做开放式音频系统，这种早期的音频系统这种基于文件系统的访问方式，这意味着对声音的操作完全可以像对普通文件那样执行open，read等操作。

​		由于OSS设计上的缺陷，导致其对混音的支持不好，再加上2002年以后，OSS成为商业不开源软件，这就催生了Linux下另一种音频系统ALSA的出现，**ALSA全称是Advanced Linux Sound Architecture**，叫做Linux系统高级音频架构，它主要为声卡提供的驱动组件，以替代原先的OSS。

​		ALSA的主要特性包括：高效地支持从消费类入门级声卡到专业级音频设备所有类型的音频接口，完全模块化的设计， 支持对称多处理（SMP）和线程安全，对OSS的向后兼容，以及提供了用户空间的alsa-lib库来简化应用程序的开发。

​		在Linux中，提供了对ALSA驱动的进一层封装：ASOC。本文基于ASOC框架。



# 具体内容

## 一、machine

​		先从最简单的部分入手。machine中，通过单板的私有数据中的link成员，决定ALSA声卡驱动的匹配信息。machine/s3c2440_uda1341.c中，首先模块入口、出口如下：

```c
static int s3c2440_uda1341_init(void)
{
	platform_set_drvdata(&asoc_dev, &myalsa_card);
    platform_device_register(&asoc_dev);    
    return 0;
}

static void s3c2440_uda1341_exit(void)
{
    platform_device_unregister(&asoc_dev);
}

module_init(s3c2440_uda1341_init);
module_exit(s3c2440_uda1341_exit);
MODULE_LICENSE("GPL");
```

​		可以看出，模块注册了一个asoc_dev设备，并将设备和myalsa_card绑定。具体的asoc_dev设备和snd_soc_card结构体如下：

```c
static struct platform_device asoc_dev = {
    .name         = "soc-audio",
    .id       = -1,
    .dev = { 
    	.release = asoc_release, 
	},
};

static struct snd_soc_card myalsa_card = {
	.name = "S3C2440_UDA1341",
	.owner = THIS_MODULE,
	.dai_link = &s3c2440_uda1341_dai_link,
	.num_links = 1,
};

static struct snd_soc_dai_link s3c2440_uda1341_dai_link = {
	.name = "UDA1341",
	.stream_name = "UDA1341",
	.codec_name = "uda1341-codec",
	.codec_dai_name = "uda1341-iis",
	.cpu_dai_name = "s3c2440-iis",
	.ops = &s3c2440_uda1341_ops,
	.platform_name	= "s3c2440-dma",
};

static struct snd_soc_ops s3c2440_uda1341_ops = {
	//.hw_params = s3c24xx_uda134x_hw_params,
};
```

总结一下，该模块主要做了如下两部分工作：

​		1、分配注册一个名为soc-audio的平台设备。

​		2、这个平台设备有一个私有数据 snd_soc_card，snd_soc_card里有一项snd_soc_dai_link

，snd_soc_dai_link被用来决定ASOC各部分的驱动。

​		这里给出的codec_name、codec_dai_name、cpu_dai_name、platform_name分别决定了解码芯片的DAI以及数据传输的名字，通过名字字符串来确定，所以后文的构建的驱动与该处同名。



## 二、platform

​		该层用于CPU侧的驱动，涉及两个部分：CPU的DAI接口以及CPU的数据传输（DMA）。

### 2.1 s3c2440_iis

​		入口函数中，注册了平台设备和平台驱动。注册平台设备主要是为了确定设备驱动的匹配关系，是平台设备驱动框架下很常用的小技巧，后面调用到平台设备驱动框架接口时，会用到该dev。

```c
static struct platform_device s3c2440_iis_dev = {
    .name         = "s3c2440-iis",
    .id       = -1,
    .dev = { 
    	.release = s3c2440_iis_release, 
	},
};
struct platform_driver s3c2440_iis_drv = {
	.probe		= s3c2440_iis_probe,
	.remove		= s3c2440_iis_remove,
	.driver		= {
		.name	= "s3c2440-iis",
	}
};
```

​		当设备驱动匹配，驱动的probe函数被调用，会依赖于上面注册的dev结构体，为dev绑定i2s_dai。如下所示：

```c
static int s3c2440_iis_probe(struct platform_device *pdev)
{
	return snd_soc_register_dai(&pdev->dev, &s3c2440_i2s_dai);
}
```

​		dai结构包含CPU的IIS通道所支持的格式，以及一个操作函数集合ops：

```c
static struct snd_soc_dai_driver s3c2440_i2s_dai = {
	.playback = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = S3C24XX_I2S_RATES,
		.formats = SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_S16_LE,},
	.capture = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = S3C24XX_I2S_RATES,
		.formats = SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_S16_LE,},
	.ops = &s3c2440_i2s_dai_ops,
};

static const struct snd_soc_dai_ops s3c2440_i2s_dai_ops = {
	.hw_params	= s3c2440_i2s_hw_params,
	.trigger	= s3c2440_i2s_trigger,
};
```

​		这里的iis格式，和后面将会写到的uda1341的iis格式，应该设置为匹配。在操作函数集合中，提供两个函数：

​		s3c2440_i2s_hw_params用于通过用户空间传入的参数，设置CPU侧的IIS控制器属性，为硬件相关的设置：

```c
static int s3c2440_i2s_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
    /* 根据params设置IIS控制器 */

    int tmp_fs;
    int i;
    int min = 0xffff;
    int pre = 0;
    unsigned int fs;
    struct clk *clk = clk_get(NULL, "pclk");

    /* 配置GPIO用于IIS */
    *gpecon &= ~((3<<0) | (3<<2) | (3<<4) | (3<<6) | (3<<8));
    *gpecon |= ((2<<0) | (2<<2) | (2<<4) | (2<<6) | (2<<8));
    
    
    /* bit[9] : Master clock select, 0-PCLK
     * bit[8] : 0 = Master mode
     * bit[7:6] : 10 = Transmit mode
     * bit[4] : 0-IIS compatible format
     * bit[2] : 384fs, 确定了MASTER CLOCK之后, fs = MASTER CLOCK/384
     * bit[1:0] : Serial bit clock frequency select, 32fs
     */
     
    if (params_format(params) == SNDRV_PCM_FORMAT_S16_LE)
        iis_regs->iismod = (2<<6) | (0<<4) | (1<<3) | (1<<2) | (1);
    else if (params_format(params) == SNDRV_PCM_FORMAT_S8)
        iis_regs->iismod = (2<<6) | (0<<4) | (0<<3) | (1<<2) | (1);
    else
        return -EINVAL;

    /* Master clock = PCLK/(n+1)
     * fs = Master clock / 384
     * fs = PCLK / (n+1) / 384
     */
    fs = params_rate(params);
    for (i = 0; i <= 31; i++)
    {
        tmp_fs = clk_get_rate(clk)/384/(i+1);
        if (ABS(tmp_fs, fs) < min)
        {
            min = ABS(tmp_fs, fs);
            pre = i;
        }
    }
    iis_regs->iispsr = (pre << 5) | (pre);

    /*
     * bit15 : Transmit FIFO access mode select, 1-DMA
     * bit13 : Transmit FIFO, 1-enable
     */
    iis_regs->iisfcon = (1<<15) | (1<<13);
    
    /*
     * bit[5] : Transmit DMA service request, 1-enable
     * bit[1] : IIS prescaler, 1-enable
     */
    iis_regs->iiscon = (1<<5) | (1<<1) ;

    clk_put(clk);
    
    return 0;
}
```

​		另一操作函数s3c2440_i2s_trigger，根据传入的命令，来打开、关闭IIS传输通道：

```c
static int s3c2440_i2s_trigger(struct snd_pcm_substream *substream, int cmd,
			       struct snd_soc_dai *dai)
{
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
        s3c2440_iis_start();
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	default:
        s3c2440_iis_stop();
		ret = -EINVAL;
		break;
	}
	return ret;
}

static void s3c2440_iis_start(void)
{
    iis_regs->iiscon |= (1);
}

static void s3c2440_iis_stop(void)
{
    iis_regs->iiscon &= ~(1);
}
```



### 2.2 s3c2440_dma

​		该部分涉及真正的数据传输，相对比较复杂，主要工作如下：

​		1、需要构建一个足够大的环形缓冲区buffer，由两个指针控制：分别代表写入，读出（hw_ptr）。

​		2、根据用户空间传入的命令（如播放），从buffer中取出一个period。period是ASLA声卡数据传输的单位，可以理解为一次传输一段period，每一个period包含很多采样点frame，根据属性设置，1个frame包含左右两个声道（左 | 右）。

​		3、设置源、目的、长度，启动DMA传输。

​		4、传输完成，DMA中断触发，在中断处理程序中：更新hw_ptr指针，指向下一个period。

​		5、若buffer写满，则更新写入指针，指向buffer头部。



​		首先是驱动入口，前面以及介绍过了平台设备驱动的框架，这里就不多解释了：

```c
static struct platform_device s3c2440_dma_dev = {
    .name         = "s3c2440-dma",
    .id       = -1,
    .dev = { 
    	.release = s3c2440_dma_release, 
	},
};
struct platform_driver s3c2440_dma_drv = {
	.probe		= s3c2440_dma_probe,
	.remove		= s3c2440_dma_remove,
	.driver		= {
		.name	= "s3c2440-dma",
	}
};
static int s3c2440_dma_init(void)
{
    dma_regs = ioremap(DMA2_BASE_ADDR, sizeof(struct s3c_dma_regs));
    platform_device_register(&s3c2440_dma_dev);
    platform_driver_register(&s3c2440_dma_drv);
    return 0;
}

static void s3c2440_dma_exit(void)
{
    platform_device_unregister(&s3c2440_dma_dev);
    platform_driver_unregister(&s3c2440_dma_drv);
    iounmap(dma_regs);
}

module_init(s3c2440_dma_init);
module_exit(s3c2440_dma_exit);

MODULE_LICENSE("GPL");
```

​		当设备和驱动匹配，调用驱动的probe函数，为设备注册并绑定dma：

```c
static int s3c2440_dma_probe(struct platform_device *pdev)
{
	return snd_soc_register_platform(&pdev->dev, &s3c2440_dma_platform);
}

static struct snd_soc_platform_driver s3c2440_dma_platform = {
	.ops		= &s3c2440_dma_ops,
	.pcm_new	= s3c2440_dma_new,		//分配缓存
	.pcm_free	= s3c2440_dma_free,		//释放缓存
};

static struct snd_pcm_ops s3c2440_dma_ops = {
	.open		= s3c2440_dma_open,		//入口，申请dma中断
	.close		= s3c2440_dma_close,	//退出
	.ioctl		= snd_pcm_lib_ioctl,	//暂空
	.hw_params	= s3c2440_dma_hw_params,//根据用户传入参数，设置dma(如用户使用多大的缓存)
	.prepare    = s3c2440_dma_prepare,	//dma传输前的准备，复位信息，加载第一个period
	.trigger	= s3c2440_dma_trigger,	//根据用户传入的命令，启动、停止数据传输
	.pointer	= s3c2440_dma_pointer,	//驱动读写指针相关
};
```

​		首先调用s3c2440_dma_new创建缓冲区，声明info结构体，并构造全局变量，用于存储dma信息，各个成员含义如下注释所示：

```c
struct s3c2440_dma_info {
    unsigned int buf_max_size;		//分配的缓冲区大小
    unsigned int buffer_size;		//用户决定的实际使用大小
    unsigned int period_size;		//缓冲区一个消息（字段）的大小
    unsigned int phy_addr;			//物理地址
    unsigned int virt_addr;			//虚拟地址
    unsigned int dma_ofs;			//偏移地址，用于寻址下一段period
    unsigned int be_running;		//dma当前传输状态
};
static struct s3c2440_dma_info playback_dma_info;

static int s3c2440_dma_new(struct snd_soc_pcm_runtime *rtd)
{
	... ...
    /* 1. 分配DMA BUFFER */
	playback_dma_info.virt_addr = (unsigned int)dma_alloc_writecombine(pcm->card->dev, s3c2440_dma_hardware.buffer_bytes_max,&playback_dma_info.phy_addr, GFP_KERNEL);//虚拟地址

    playback_dma_info.buf_max_size = s3c2440_dma_hardware.buffer_bytes_max;    	
    buf->dev.type = SNDRV_DMA_TYPE_DEV;    	
    buf->dev.dev = pcm->card->dev;    	
    buf->private_data = NULL;        
    buf->area = playback_dma_info.virt_addr;  
    buf->bytes = playback_dma_info.buf_max_size;    
}
```

​		缓存的分配，用到了硬件参数是提前设置好的，在结构体s3c2440_dma_hardware中。

​		在open函数中，设置DMA硬件，为DMA申请中断：

```c
static int s3c2440_dma_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
    int ret;

    /* 设置属性 */
	snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);
	snd_soc_set_runtime_hwparams(substream, &s3c2440_dma_hardware);

    /* 注册中断 */
    ret = request_irq(IRQ_DMA2, s3c2440_dma2_irq, IRQF_DISABLED, "myalsa for playback", substream);
    if (ret)
    {
        printk("request_irq error!\n");
        return -EIO;
    }
	return 0;
}
```

​		DMA中断服务程序调用，代表DMA传输一次period完成。接下来需要更新数据读取的指针，指向下一个period，若超出buffer实际大小，则指向开始（偏移地址归零）。并判断buffer中是否还有数据，若无，则调用tigger停止。最后加载下一个period，并启动传输：

```c
static irqreturn_t s3c2440_dma2_irq(int irq, void *devid)
{
    struct snd_pcm_substream *substream = devid;
        
    /* 更新状态信息 */
    playback_dma_info.dma_ofs += playback_dma_info.period_size;
    if (playback_dma_info.dma_ofs >= playback_dma_info.buffer_size)
        playback_dma_info.dma_ofs = 0;
    
    /* 更新hw_ptr等信息,
     * 并且判断:如果buffer里没有数据了,则调用trigger来停止DMA 
     */
    snd_pcm_period_elapsed(substream);  

    if (playback_dma_info.be_running)
    {
        /* 如果还有数据
         * 1. 加载下一个period 
         * 2. 再次启动DMA传输
         */
        load_dma_period();
        s3c2440_dma_start();
    }
    return IRQ_HANDLED;
}
```

​		调用ops下的s3c2440_dma_hw_params，通过用户空间传入参数设置DMA，做一些初始化工作：

```c
static int s3c2440_dma_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned long totbytes = params_buffer_bytes(params);	//从参数中解析出分配的缓存大小
    
    /* 根据params设置DMA */
	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);

    /* s3c2440_dma_new分配了很大的DMA BUFFER
     * params决定使用多大
     */
	runtime->dma_bytes            = totbytes;
    playback_dma_info.buffer_size = totbytes;
    playback_dma_info.period_size = params_period_bytes(params);

    return 0;
}
```

​		DMA准备就绪，将数据加载到DMA（将源、目的、长度告知DMA），等待命令准备传输：

```c
static int s3c2440_dma_prepare(struct snd_pcm_substream *substream)
{
  /* 复位各种状态信息 */
    playback_dma_info.dma_ofs = 0;
    playback_dma_info.be_running = 0;
    
    /* 加载第1个period */
    load_dma_period();

	return 0;
}
```

​		根据用户传入命令，启动、关闭传输：

```c
static int s3c2440_dma_trigger(struct snd_pcm_substream *substream, int cmd)
{
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
        /* 启动DMA传输 */
        playback_dma_info.be_running = 1;
        s3c2440_dma_start();
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
        /* 停止DMA传输 */
        playback_dma_info.be_running = 0;
        s3c2440_dma_stop();
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}
```



## 三、codec

​		包含1个驱动：uda1341.c，用于codec解码芯片相关的工作，从入口开始：

```c
static struct platform_device uda1341_dev = {
    .name         = "uda1341-codec",
    .id       = -1,
    .dev = { 
    	.release = uda1341_dev_release, 
	},
};
struct platform_driver uda1341_drv = {
	.probe		= uda1341_probe,
	.remove		= uda1341_remove,
	.driver		= {
		.name	= "uda1341-codec",
	}
};

static int uda1341_init(void)
{
    gpbcon = ioremap(0x56000010, 4);
    gpbdat = ioremap(0x56000014, 4);
    
    platform_device_register(&uda1341_dev);
    platform_driver_register(&uda1341_drv);
    return 0;
}

static void uda1341_exit(void)
{
    platform_device_unregister(&uda1341_dev);
    platform_driver_unregister(&uda1341_drv);

    iounmap(gpbcon);
    iounmap(gpbdat);
}

module_init(uda1341_init);
module_exit(uda1341_exit);

MODULE_LICENSE("GPL");
```

​		注册平台驱动，此处还注册了一个几乎为空的设备，主要是为了确定平台设备和平台驱动的匹配关系，后面的平台接口要用到设备结构体。没有别的意义，这只是一个平台设备驱动的小技巧。

​		注册完成后，设备与驱动匹配，调用驱动的probe函数：


```c
static int uda1341_probe(struct platform_device *pdev)
{
	return snd_soc_register_codec(&pdev->dev,
			&soc_codec_dev_uda1341, &uda1341_dai, 1);
}
```

​		这里调用内核的ASOC接口，注册并为设备绑定了属性设置通道以及数据传输通道。具体来讲，dai通道是属性设置通道，表明了UDA1341支持的格式，以及属性设置的操作。分为两个逻辑设备：录音和播放。如下所示：


```c
static struct snd_soc_dai_driver uda1341_dai = {
	.name = "uda1341-iis",
	/* playback capabilities */
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = UDA134X_RATES,
		.formats = UDA134X_FORMATS,
	},
	/* capture capabilities */
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = UDA134X_RATES,
		.formats = UDA134X_FORMATS,
	},
	/* pcm operations */
	.ops = &uda1341_dai_ops,
};
static const struct snd_soc_dai_ops uda1341_dai_ops = {
	.hw_params	= uda1341_hw_params,
};
```

​			本来可以在操作函数的uda1341_hw_params中，根据用户空间传入的参数，对硬件进行相关的初始化和设置。但是考虑到将初始化作为一个整体，所以codec硬件初始化放到了uda1341_init_regs里，设置好时钟、格式等参数。

​		以上是codec属性设置通道的对象，下面是数据传输的对象：


```c
static struct snd_soc_codec_driver soc_codec_dev_uda1341 = {
    .probe = uda1341_soc_probe,
    
    /* UDA1341的寄存器不支持读操作
     * 要知道某个寄存器的当前值,
     * 只能在写入时保存起来
     */
	.reg_cache_size = sizeof(uda1341_reg),
	.reg_word_size = sizeof(u8),
	.reg_cache_default = uda1341_reg,
	.reg_cache_step = 1,
	.read  = uda1341_read_reg_cache,
	.write = uda1341_write_reg,  /* 写寄存器 */
};
```

​		这里需要注意一下uda1341的寄存器读写操作，写操作正常，但是uda1341不支持读操作，所以需要在写寄存器时，将值缓存。

​		当soc_codec_dev_uda1341数据传输通道匹配，调用probe：


```c
static int uda1341_soc_probe(struct snd_soc_codec *codec)
{
    int ret;
    uda1341_init_regs(codec);
    
	ret = snd_soc_add_codec_controls(codec, &uda1341_vol_control, 1);
    return ret;
}
```

​		probe首先调用uda1341_init_regs初始化了codec芯片硬件相关寄存器，包括了数据传输以及属性设置两个通道。并且添加一个音量控制的对象，如下所示：

```c
static const struct snd_kcontrol_new uda1341_vol_control = 
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, 
    .name = "Master Playback Volume", 
	.info = uda1341_info_vol, 	//查询音量信息，如调整区间，当前音量
	.get  = uda1341_get_vol,	//获取
	.put  = uda1341_put_vol, 	//设置
};
```

​		这里介绍一下音量设置的方法。

```c
int uda1341_info_vol(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 2;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 63;
	return 0;
}

int uda1341_get_vol(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

    ucontrol->value.integer.value[1] = \
	ucontrol->value.integer.value[0] = 63 - snd_soc_read(codec, UDA1341_DATA00);
	return 0;
}

int uda1341_put_vol(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	unsigned int val;

	val = 63 - ucontrol->value.integer.value[0];

    snd_soc_write(codec, UDA1341_DATA00, val);
    
	return 0;
}
```

​		需要注意，uda1341的音量控制和应用是相反的，即硬件的取值区间是0 - 63，63为最小音量，0为最大音量。





# 添加入内核

​		将写好的machine、codec两个模块添加进内核，并支持在menu中配置。

Kconfig

```c
config SND_SOC_SAMSUNG_S3C24XX_WM8976
	tristate "SoC I2S Audio support WM8976 wired to a S3C24XX"
	depends on SND_SOC_SAMSUNG && ARCH_S3C24XX
	select SND_S3C24XX_I2S
	select SND_SOC_L3
	select SND_SOC_WM8976
```

Makefile

```c
snd-soc-s3c24xx-wm8976-objs := s3c2440_wm8976.o
```



# 总结

​		驱动分为三个部分：machine、platform、codec。

​		1、machine注册sound设备，设备私有数据规定了ALSA声卡传输：CPU侧的DAI接口（IIS）、CPU侧的数据传输接口（DMA2）、codec解码芯片及其DAI（IIS）。

​		

​		2、platform中，分为两个部分：s3c2440_iis、s3c2440_dma

​				1）、s3c2440_iis声明了CPU侧使用的IIS格式，提供两个操作函数：硬件初始化设置，以及IIS属性设置的触发：包括开始和停止。

​				2）、s3c2440_dma提供操作函数以及缓存的分配和释放，在操作函数中，可以：初始化DMA，并注册DMA中断。在DMA中断服务程序中，加载，并开始下一个数据传输。根据用户参数设置DMA通道。DMA数据的预加载。根据用户命令，触发传输。

​	

​		3、codec注册解码芯片uda1341，以及其使用的dai格式（IIS），提供解码芯片的初始化，绑定了音量查询、调节功能。























