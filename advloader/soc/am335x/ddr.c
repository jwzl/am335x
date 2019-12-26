#include "types.h"
#include "error.h"
#include "hw/hw_types.h"
#include "am335x/hw_emif4d.h"
#include "hw/soc_am335x.h"
#include "am335x/ddr.h"
#include "am335x/hw_cm_per.h"
#include "am335x/hw_cm_device.h"
#include "hw/hw_control_am335x.h"
#include "board.h"
#include "soc.h"
#include "device.h"
#include "chipdb.h"

/*
 * \brief This function initializes the EMIF
 */
void am335x_emif_init(void)
{
    HW_WR_FIELD32_RAW(SOC_CM_PER_REGS + CM_PER_EMIF_FW_CLKCTRL,
        CM_PER_EMIF_FW_CLKCTRL_MODULEMODE,
        CM_PER_EMIF_FW_CLKCTRL_MODULEMODE_SHIFT,
        CM_PER_EMIF_FW_CLKCTRL_MODULEMODE_ENABLE);

    HW_WR_FIELD32_RAW(SOC_CM_PER_REGS + CM_PER_EMIF_CLKCTRL,
        CM_PER_EMIF_CLKCTRL_MODULEMODE,
        CM_PER_EMIF_CLKCTRL_MODULEMODE_SHIFT,
        CM_PER_EMIF_CLKCTRL_MODULEMODE_ENABLE);

    while((CM_PER_L3_CLKSTCTRL_CLKACTIVITY_EMIF_GCLK_ACT !=
        HW_RD_FIELD32_RAW(SOC_CM_PER_REGS + CM_PER_L3_CLKSTCTRL,
        CM_PER_L3_CLKSTCTRL_CLKACTIVITY_EMIF_GCLK,
        CM_PER_L3_CLKSTCTRL_CLKACTIVITY_EMIF_GCLK_SHIFT)) ||
        (CM_PER_L3_CLKSTCTRL_CLKACTIVITY_L3_GCLK_ACT !=
        HW_RD_FIELD32_RAW(SOC_CM_PER_REGS + CM_PER_L3_CLKSTCTRL,
        CM_PER_L3_CLKSTCTRL_CLKACTIVITY_L3_GCLK,
        CM_PER_L3_CLKSTCTRL_CLKACTIVITY_L3_GCLK_SHIFT)));
}

static void am335x_vtp_enable(void)
{
    /* Enable VTP */
    HW_WR_FIELD32_RAW(SOC_CONTROL_REGS + CONTROL_VTP_CTRL, CONTROL_VTP_CTRL_ENABLE,
        CONTROL_VTP_CTRL_ENABLE_SHIFT, 1U);
    HW_WR_FIELD32_RAW(SOC_CONTROL_REGS + CONTROL_VTP_CTRL, CONTROL_VTP_CTRL_CLRZ,
        CONTROL_VTP_CTRL_CLRZ_SHIFT, 0U);
    HW_WR_FIELD32_RAW(SOC_CONTROL_REGS + CONTROL_VTP_CTRL, CONTROL_VTP_CTRL_CLRZ,
        CONTROL_VTP_CTRL_CLRZ_SHIFT, 1U);

    while(1U != HW_RD_FIELD32_RAW(SOC_CONTROL_REGS + CONTROL_VTP_CTRL,
        CONTROL_VTP_CTRL_READY, CONTROL_VTP_CTRL_READY_SHIFT));
}

static void am335x_ddr_phy_init(sblPlatformDdrEmifPhyCfg_t *pDdrEmifPhyCfg, uint32_t memType)
{
    volatile uint32_t regVal;

    /* Enable VTP */
    am335x_vtp_enable();

	/* DDR PHY CMD Register configuration */
    regVal = pDdrEmifPhyCfg->slaveRatio;
    HW_WR_REG32((CMD0_SLAVE_RATIO_0)   , regVal);
    HW_WR_REG32((CMD1_SLAVE_RATIO_0)   , regVal);
    HW_WR_REG32((CMD2_SLAVE_RATIO_0)   , regVal);

    if(SBL_PLATFORM_MEM_TYPE_DDR2 == memType)
    {
        regVal = pDdrEmifPhyCfg->slaveForce;
        HW_WR_REG32((CMD0_SLAVE_FORCE_0)   , regVal);
        HW_WR_REG32((CMD1_SLAVE_FORCE_0)   , regVal);
	HW_WR_REG32((CMD2_SLAVE_FORCE_0)   , regVal);

	regVal = pDdrEmifPhyCfg->slaveDelay;
	HW_WR_REG32((CMD0_SLAVE_DELAY_0)   , regVal);
	HW_WR_REG32((CMD1_SLAVE_DELAY_0)   , regVal);
	HW_WR_REG32((CMD2_SLAVE_DELAY_0)   , regVal);
    }

    regVal = pDdrEmifPhyCfg->invertClkout;
    HW_WR_REG32((CMD0_INVERT_CLKOUT_0)   , regVal);
    HW_WR_REG32((CMD1_INVERT_CLKOUT_0)   , regVal);
    HW_WR_REG32((CMD2_INVERT_CLKOUT_0)   , regVal);

    /* DDR PHY DATA Register configuration */
    regVal = pDdrEmifPhyCfg->rdDqsSlaveRatio;
    HW_WR_REG32((DATA0_RD_DQS_SLAVE_RATIO_0)  , regVal);
    HW_WR_REG32((DATA1_RD_DQS_SLAVE_RATIO_0)  , regVal);

    regVal = pDdrEmifPhyCfg->wrDqsSlaveRatio;
    HW_WR_REG32((DATA0_WR_DQS_SLAVE_RATIO_0)  , regVal);
    HW_WR_REG32((DATA1_WR_DQS_SLAVE_RATIO_0)  , regVal);

    regVal = pDdrEmifPhyCfg->fifoWeSlaveRatio;
    HW_WR_REG32((DATA0_FIFO_WE_SLAVE_RATIO_0)  , regVal);
    HW_WR_REG32((DATA1_FIFO_WE_SLAVE_RATIO_0)  , regVal);

    regVal = pDdrEmifPhyCfg->wrDataSlaveRatio;
    HW_WR_REG32((DATA0_WR_DATA_SLAVE_RATIO_0)  , regVal);
    HW_WR_REG32((DATA1_WR_DATA_SLAVE_RATIO_0)  , regVal);
}

void am335x_ddr_config(uint32_t dynPwrDown,sblPlatformDdrCfg_t *pDdrCfg, uint32_t memType)
{
    volatile uint32_t delay = 5000;
    volatile uint32_t regVal, structVal;
    sblPlatformDdrCtrlCfg_t *pDdrCtrlCfg = &pDdrCfg->ddrCtrlCfg;
    sblPlatformDdrEmifCfg_t *pDdrEmifCfg = &pDdrCfg->ddrEmifCfg;

    /* DDR2 Phy Initialization */
    am335x_ddr_phy_init(&pDdrCfg->ddrEmifPhyCfg, memType);

    regVal = pDdrCtrlCfg->addrIoCtrl;
    HW_WR_REG32((SOC_CONTROL_REGS + CONTROL_DDR_CMD_IOCTRL(0)) , regVal);
    HW_WR_REG32((SOC_CONTROL_REGS + CONTROL_DDR_CMD_IOCTRL(1)) , regVal);
    HW_WR_REG32((SOC_CONTROL_REGS + CONTROL_DDR_CMD_IOCTRL(2)) , regVal);

    regVal = pDdrCtrlCfg->dataIoCtrl;
    HW_WR_REG32((SOC_CONTROL_REGS + CONTROL_DDR_DATA_IOCTRL(0)) , regVal);
    HW_WR_REG32((SOC_CONTROL_REGS + CONTROL_DDR_DATA_IOCTRL(1)) , regVal);

    /* IO to work for DDR */
    regVal = HW_RD_REG32(SOC_CONTROL_REGS + CONTROL_DDR_IO_CTRL);
    structVal = pDdrCtrlCfg->ddrIoCtrl;
    regVal &= structVal;
    HW_WR_REG32((SOC_CONTROL_REGS + CONTROL_DDR_IO_CTRL), regVal);

    HW_WR_FIELD32_RAW(SOC_CONTROL_REGS + CONTROL_DDR_CKE_CTRL,
        CONTROL_DDR_CKE_CTRL_DDR_CKE_CTRL, CONTROL_DDR_CKE_CTRL_DDR_CKE_CTRL_SHIFT, 1U);

    regVal = pDdrEmifCfg->ddrPhyCtrl;
    HW_WR_REG32((SOC_EMIF_0_REGS + EMIF_DDR_PHY_CTRL_1) , regVal);

    /* Dynamic Power Down */
    if(TRUE == dynPwrDown)
    {
        regVal = HW_RD_REG32(SOC_EMIF_0_REGS + EMIF_DDR_PHY_CTRL_1);
        structVal = pDdrEmifCfg->ddrPhyCtrlDyPwrdn;
        regVal |= structVal;
        HW_WR_REG32((SOC_EMIF_0_REGS + EMIF_DDR_PHY_CTRL_1), regVal);
    }
    regVal = pDdrEmifCfg->ddrPhyCtrl;
    HW_WR_REG32((SOC_EMIF_0_REGS + EMIF_DDR_PHY_CTRL_1_SHDW) ,
                                                 regVal);

    /* Dynamic Power Down */
    if(TRUE == dynPwrDown)
    {
        regVal = HW_RD_REG32(SOC_EMIF_0_REGS + EMIF_DDR_PHY_CTRL_1_SHDW);
        structVal = pDdrEmifCfg->ddrPhyCtrlDyPwrdn;
        regVal |= structVal;
        HW_WR_REG32((SOC_EMIF_0_REGS + EMIF_DDR_PHY_CTRL_1_SHDW), regVal);
    }
    regVal = pDdrEmifCfg->ddrPhyCtrl;
    HW_WR_REG32((SOC_EMIF_0_REGS + EMIF_DDR_PHY_CTRL_2) , regVal);

    regVal = pDdrEmifCfg->sdramTim1;
    HW_WR_REG32((SOC_EMIF_0_REGS + EMIF_SDRAM_TIM_1)      , regVal);
    HW_WR_REG32((SOC_EMIF_0_REGS + EMIF_SDRAM_TIM_1_SHDW) , regVal);

    regVal = pDdrEmifCfg->sdramTim2;
    HW_WR_REG32((SOC_EMIF_0_REGS + EMIF_SDRAM_TIM_2)      , regVal);
    HW_WR_REG32((SOC_EMIF_0_REGS + EMIF_SDRAM_TIM_2_SHDW) , regVal);

    regVal = pDdrEmifCfg->sdramTim3;
    HW_WR_REG32((SOC_EMIF_0_REGS + EMIF_SDRAM_TIM_3)      , regVal);
    HW_WR_REG32((SOC_EMIF_0_REGS + EMIF_SDRAM_TIM_3_SHDW) , regVal);

    regVal = pDdrEmifCfg->sdramCfg;
    HW_WR_REG32((SOC_EMIF_0_REGS + EMIF_SDRAM_CONFIG)     , regVal);

    regVal = pDdrEmifCfg->sdramRefCtrl1;
    HW_WR_REG32((SOC_EMIF_0_REGS + EMIF_SDRAM_REF_CTRL)   , regVal);
    HW_WR_REG32((SOC_EMIF_0_REGS + EMIF_SDRAM_REF_CTRL_SHDW) , regVal);

    if(SBL_PLATFORM_MEM_TYPE_DDR2 == memType)
    {
        while(delay--);
        regVal = pDdrEmifCfg->sdramRefCtrl2;
        HW_WR_REG32((SOC_EMIF_0_REGS + EMIF_SDRAM_REF_CTRL) , regVal);
        HW_WR_REG32((SOC_EMIF_0_REGS + EMIF_SDRAM_REF_CTRL_SHDW) , regVal);
    }

    if(SBL_PLATFORM_MEM_TYPE_DDR3 == memType)
    {
        regVal = pDdrEmifCfg->zqConfig;
        HW_WR_REG32((SOC_EMIF_0_REGS + EMIF_ZQ_CONFIG)     , regVal);
    }

    regVal = pDdrEmifCfg->sdramCfg;
    HW_WR_REG32((SOC_EMIF_0_REGS + EMIF_SDRAM_CONFIG)     , regVal);

    /* The CONTROL_SECURE_EMIF_SDRAM_CONFIG register exports SDRAM configuration
       information to the EMIF */
    HW_WR_REG32((SOC_CONTROL_REGS + CONTROL_SECURE_EMIF_SDRAM_CONFIG) , regVal);
}
