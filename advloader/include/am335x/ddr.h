#ifndef _AM335x_DDR_H_
#define _AM335x_DDR_H_

#ifdef __cplusplus
extern "C" {
#endif

/* TODO : These are not there in the control module header file */
#define DDR_PHY_CTRL_REGS                  (SOC_CONTROL_REGS + 0x2000)
#define CMD0_SLAVE_RATIO_0                 (DDR_PHY_CTRL_REGS + 0x1C)
#define CMD0_SLAVE_FORCE_0                 (DDR_PHY_CTRL_REGS + 0x20)
#define CMD0_SLAVE_DELAY_0                 (DDR_PHY_CTRL_REGS + 0x24)
#define CMD0_LOCK_DIFF_0                   (DDR_PHY_CTRL_REGS + 0x28)
#define CMD0_INVERT_CLKOUT_0               (DDR_PHY_CTRL_REGS + 0x2C)
#define CMD1_SLAVE_RATIO_0                 (DDR_PHY_CTRL_REGS + 0x50)
#define CMD1_SLAVE_FORCE_0                 (DDR_PHY_CTRL_REGS + 0x54)
#define CMD1_SLAVE_DELAY_0                 (DDR_PHY_CTRL_REGS + 0x58)
#define CMD1_LOCK_DIFF_0                   (DDR_PHY_CTRL_REGS + 0x5C)
#define CMD1_INVERT_CLKOUT_0               (DDR_PHY_CTRL_REGS + 0x60)
#define CMD2_SLAVE_RATIO_0                 (DDR_PHY_CTRL_REGS + 0x84)
#define CMD2_SLAVE_FORCE_0                 (DDR_PHY_CTRL_REGS + 0x88)
#define CMD2_SLAVE_DELAY_0                 (DDR_PHY_CTRL_REGS + 0x8C)
#define CMD2_LOCK_DIFF_0                   (DDR_PHY_CTRL_REGS + 0x90)
#define CMD2_INVERT_CLKOUT_0               (DDR_PHY_CTRL_REGS + 0x94)
#define DATA0_RD_DQS_SLAVE_RATIO_0         (DDR_PHY_CTRL_REGS + 0xC8)
#define DATA0_RD_DQS_SLAVE_RATIO_1         (DDR_PHY_CTRL_REGS + 0xCC)
#define DATA0_WR_DQS_SLAVE_RATIO_0         (DDR_PHY_CTRL_REGS + 0xDC)
#define DATA0_WR_DQS_SLAVE_RATIO_1         (DDR_PHY_CTRL_REGS + 0xE0)
#define DATA0_WRLVL_INIT_RATIO_0           (DDR_PHY_CTRL_REGS + 0xF0)
#define DATA0_WRLVL_INIT_RATIO_1           (DDR_PHY_CTRL_REGS + 0xF4)
#define DATA0_GATELVL_INIT_RATIO_0         (DDR_PHY_CTRL_REGS + 0xFC)
#define DATA0_GATELVL_INIT_RATIO_1         (DDR_PHY_CTRL_REGS + 0x100)
#define DATA0_FIFO_WE_SLAVE_RATIO_0        (DDR_PHY_CTRL_REGS + 0x108)
#define DATA0_FIFO_WE_SLAVE_RATIO_1        (DDR_PHY_CTRL_REGS + 0x10C)
#define DATA0_WR_DATA_SLAVE_RATIO_0        (DDR_PHY_CTRL_REGS + 0x120)
#define DATA0_WR_DATA_SLAVE_RATIO_1        (DDR_PHY_CTRL_REGS + 0x124)
#define DATA0_USE_RANK0_DELAYS_0           (DDR_PHY_CTRL_REGS + 0x134)
#define DATA0_LOCK_DIFF_0                  (DDR_PHY_CTRL_REGS + 0x138)
#define DATA1_RD_DQS_SLAVE_RATIO_0         (DDR_PHY_CTRL_REGS + 0x16c)
#define DATA1_RD_DQS_SLAVE_RATIO_1         (DDR_PHY_CTRL_REGS + 0x170)
#define DATA1_WR_DQS_SLAVE_RATIO_0         (DDR_PHY_CTRL_REGS + 0x180)
#define DATA1_WR_DQS_SLAVE_RATIO_1         (DDR_PHY_CTRL_REGS + 0x184)
#define DATA1_WRLVL_INIT_RATIO_0           (DDR_PHY_CTRL_REGS + 0x194)
#define DATA1_WRLVL_INIT_RATIO_1           (DDR_PHY_CTRL_REGS + 0x198)
#define DATA1_GATELVL_INIT_RATIO_0         (DDR_PHY_CTRL_REGS + 0x1a0)
#define DATA1_GATELVL_INIT_RATIO_1         (DDR_PHY_CTRL_REGS + 0x1a4)
#define DATA1_FIFO_WE_SLAVE_RATIO_0        (DDR_PHY_CTRL_REGS + 0x1ac)
#define DATA1_FIFO_WE_SLAVE_RATIO_1        (DDR_PHY_CTRL_REGS + 0x1b0)
#define DATA1_WR_DATA_SLAVE_RATIO_0        (DDR_PHY_CTRL_REGS + 0x1c4)
#define DATA1_WR_DATA_SLAVE_RATIO_1        (DDR_PHY_CTRL_REGS + 0x1c8)
#define DATA1_USE_RANK0_DELAYS_0           (DDR_PHY_CTRL_REGS + 0x1d8)
#define DATA1_LOCK_DIFF_0                  (DDR_PHY_CTRL_REGS + 0x1dc)


/**< \brief Enumerates the memory types. */
typedef enum sblPlatformMemoryType
{
    SBL_PLATFORM_MEM_TYPE_MIN,
    /**< Invalid memory type. */
    SBL_PLATFORM_MEM_TYPE_INVALID = SBL_PLATFORM_MEM_TYPE_MIN,
    /**< Invalid memory type. */
    SBL_PLATFORM_MEM_TYPE_DDR2,
    /**< LPDDR2 memory type. */
    SBL_PLATFORM_MEM_TYPE_DDR3,
    /**< DDR3 memory type. */
    SBL_PLATFORM_MEM_TYPE_MAX = SBL_PLATFORM_MEM_TYPE_DDR3
    /**< Maximum value of memory type. */
}sblPlatformMemoryType_t;

/**< \brief Structure holding the values of control registers. */
typedef struct sblPlatformDdrCtrlCfg
{
    uint32_t addrIoCtrl;
    /**<
     *  DDR addr io control to configure
     *  -# slew rate of clock IO pads
     *  -# pull up and pull down output impedence of clock IO pads
     *  -# addr/cmd IO Pads output Slew Rate
     *  -# program addr/cmd IO Pad Pull up Pull down output Impedance
     */
    uint32_t dataIoCtrl;
    /**<
     *  DDR data0/1/2 io control to configure
     *  -# controls WD0 & WD1 of DQS0/1/2.
     *  -# controls WD0 & WD1 of DM0/1/2.
     *  -# controls WD0 & WD1 of DDR data pins.
     *  -# clock IO Pads (DQS(0/1/2)/DQ(0/1/2)S#) output Slew Rate.
     *  -# clock IO Pads (DQS/DQS#) Pull up Pull down output Impedance.
     *  -# data IO Pads output Slew Rate.
     *  -# data IO Pad Pull up Pull down output Impedance.
     * Please refer to DDR IO buffer spec for details
     */
    uint32_t ddrIoCtrl;
    /**<
     *  DDR io control
     */
}sblPlatformDdrCtrlCfg_t;

/**< \brief Structure holding the values of DDR Emif registers. */
typedef struct sblPlatformDdrEmifCfg
{
    uint32_t ddrPhyCtrl;
    /**< DDR phy control register value. */
    uint32_t ddrPhyCtrlDyPwrdn;
    /**< DDR phy control dynamic power down register value. */
    uint32_t sdramTim1;
    /**< SDRAM timing1 register value. */
    uint32_t sdramTim2;
    /**< SDRAM timing2 register value. */
    uint32_t sdramTim3;
    /**< SDRAM timing3 register value. */
    uint32_t sdramCfg;
    /**< SDRAM config register value. */
    uint32_t sdramRefCtrl1;
    /**< SDRAM ref control register value 1. */
    uint32_t sdramRefCtrl2;
    /**< SDRAM ref control register value 2. */
    uint32_t zqConfig;
    /**< ZQ config register value. */
}sblPlatformDdrEmifCfg_t;

/**< \brief Structure holding the parameters to configure EMIF phy registers. */
typedef struct sblPlatformDdrEmifPhyCfg
{
    /**< COMMAND */
    uint32_t slaveRatio;
    /**< slave ratio. */
    uint32_t slaveForce;
    /**< slave force. */
    uint32_t slaveDelay;
    /**< slave delay. */
    uint32_t invertClkout;
    /**< invert clockout. */

    /**< DATA */
    uint32_t rdDqsSlaveRatio;
    /**< Read DQS slave ratio. */
    uint32_t wrDqsSlaveRatio;
    /**< Write DQS slave ratio. */
    uint32_t fifoWeSlaveRatio;
    /**< FIFO write enable slave ratio. */
    uint32_t wrDataSlaveRatio;
    /**< writer data slave ratio. */
}sblPlatformDdrEmifPhyCfg_t;

/**< \brief Structure holding the ddr configurations. */
typedef struct sblPlatformDdrCfg
{
    sblPlatformDdrCtrlCfg_t    ddrCtrlCfg;
    /**< DDR ctrl config registers. */
    sblPlatformDdrEmifCfg_t    ddrEmifCfg;
    /**< DDR Emif config registers. */
    sblPlatformDdrEmifPhyCfg_t ddrEmifPhyCfg;
    /**< EMIF phy config registers. */
}sblPlatformDdrCfg_t;


void am335x_emif_init(void);
void am335x_ddr_config(uint32_t dynPwrDown,sblPlatformDdrCfg_t *pDdrCfg, uint32_t memType);
#ifdef __cplusplus
}
#endif

#endif	/*_AM335x_DDR_H_*/
