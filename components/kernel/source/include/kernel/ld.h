#ifndef __KERNEL_LD_H
#define __KERNEL_LD_H

extern unsigned long __RAM_BASE;
extern unsigned long __text_start;
extern unsigned long __CTOR_END__;
extern unsigned long __DTOR_END__;
extern unsigned long __text_end;
extern unsigned long __runtime_reloc_start;
extern unsigned long __runtime_reloc_stop;
extern unsigned long __rodata_start;
extern unsigned long __rodata_end;
extern unsigned long __data_start;
extern unsigned long __data_end;
extern unsigned long _init_start;
extern unsigned long _module_init_start;
extern unsigned long _module_init_core_start;
extern unsigned long _module_init_core_end;
extern unsigned long _module_init_postcore_start;
extern unsigned long _module_init_postcore_end;
extern unsigned long _module_init_arch_start;
extern unsigned long _module_init_arch_end;
extern unsigned long _module_init_system_start;
extern unsigned long _module_init_system_end;
extern unsigned long _module_init_driver_start;
extern unsigned long _module_init_driver_end;
extern unsigned long _module_init_driverlate_start;
extern unsigned long _module_init_driverlate_end;
extern unsigned long _module_init_others_start;
extern unsigned long _module_init_others_end;
extern unsigned long _module_init_end;
extern unsigned long _service_init_start;
extern unsigned long _service_init_end;
extern unsigned long _console_init_start;
extern unsigned long _console_init_end;
extern unsigned long _runner_init_start;
extern unsigned long _runner_init_end;
extern unsigned long _test_case_init_start;
extern unsigned long _test_case_init_end;
extern unsigned long _padec_start;
extern unsigned long _padec_end;
extern unsigned long _pvdec_start;
extern unsigned long _pvdec_end;
extern unsigned long _deca_audio_stream_struct_start;
extern unsigned long _deca_audio_stream_struct_end;
extern unsigned long _audio_init_struct_start;
extern unsigned long _audio_init_struct_end;
extern unsigned long _init_end;
extern unsigned long __sdata_start;
extern unsigned long __sdata_end;
extern unsigned long __sbss_start;
extern unsigned long _sbss;
extern unsigned long __sbss_end;
extern unsigned long __bss_start;
extern unsigned long _ebss;
extern unsigned long __bss_end;

extern unsigned long IRC0;
extern unsigned long UART0;
extern unsigned long UART1;
extern unsigned long UART2;
extern unsigned long UART3;
extern unsigned long SFSPI0;
extern unsigned long SPIMASTER;
extern unsigned long MMBOX0;
extern unsigned long MMBOX1;
extern unsigned long MMBOX2;
extern unsigned long MMBOX3;
extern unsigned long MMBOX4;
extern unsigned long MMBOX5;
extern unsigned long MMBOX6;
extern unsigned long MMBOX7;
extern unsigned long SMBOX0;
extern unsigned long SMBOX1;
extern unsigned long SMBOX2;
extern unsigned long SMBOX3;
extern unsigned long SMBOX4;
extern unsigned long SMBOX5;
extern unsigned long SMBOX6;
extern unsigned long SMBOX7;
extern unsigned long TIMER00;
extern unsigned long TIMER10;
extern unsigned long TIMER20;
extern unsigned long TIMER30;
extern unsigned long TIMER40;
extern unsigned long TIMER50;
extern unsigned long TIMER60;
extern unsigned long TIMER70;
extern unsigned long HWSPINLOCK;
extern unsigned long SND0;
extern unsigned long SND0_DAC;
extern unsigned long SYSIO0;
extern unsigned long MSYSIO0;
extern unsigned long PINMUXL;
extern unsigned long PINMUXB;
extern unsigned long PINMUXR;
extern unsigned long PINMUXT;
extern unsigned long GPIOLCTRL;
extern unsigned long GPIOBCTRL;
extern unsigned long GPIORCTRL;
extern unsigned long GPIOTCTRL;
extern unsigned long I2C0CTRL;
extern unsigned long I2C1CTRL;
extern unsigned long I2C2CTRL;
extern unsigned long I2C3CTRL;
extern unsigned long WDT0;
extern unsigned long PWMCTRL;
extern unsigned long ADCCTRL;
extern unsigned long USB0;
extern unsigned long USB1;

extern unsigned long GPIO_INTR;
extern unsigned long H264_INTR;
extern unsigned long DE_INTR;
extern unsigned long IHB_VIN_INTR;
extern unsigned long GE_INTR;
extern unsigned long SND_INTR;
extern unsigned long TSG_INTR;
extern unsigned long DEMUX0_INTR;
extern unsigned long DEMUX1_INTR;
extern unsigned long TSI_INTR;
extern unsigned long SD_MMC_INTR;
extern unsigned long DMA1_INTR;
extern unsigned long DMA0_INTR;
extern unsigned long HDMI_INTR;
extern unsigned long MAC_INTR;
extern unsigned long MAC_WOL_INTR;
extern unsigned long UART0_INTR;
extern unsigned long UART1_INTR;
extern unsigned long I2C0_INTR;
extern unsigned long IRC_INTR;
extern unsigned long RTC_INTR;
extern unsigned long WTD_INTR;
extern unsigned long SD_MMC_DMA_INTR;
extern unsigned long I2C1_INTR;
extern unsigned long I2C2_INTR;
extern unsigned long DRAM_INTR;
extern unsigned long I2SI_PCM_PDM_TDM_INTR;
extern unsigned long MCPU_ILB_ACK_INTR;
extern unsigned long MCPU_BUS_ERR_INTR;
extern unsigned long CPU_ILB_ACK_INTR;
extern unsigned long CPU_BUS_ERR_INTR;

extern unsigned long VIN_TBD_INTR;
extern unsigned long SPI_MASTER_INTR;
extern unsigned long SF_INTR;
extern unsigned long HDMI_PHY_INTR;
extern unsigned long SAR_ADC_INTR;
extern unsigned long TSDMA_INTR;
extern unsigned long JPEG_ENCODER_INTR;
extern unsigned long DE4K_INTR;
extern unsigned long DE4K_EXTI_INTR;
extern unsigned long MIPI_INTR;
extern unsigned long I2C3_INTR;
extern unsigned long DESCRAMBLER_INTR;
extern unsigned long HRX_INTR;
extern unsigned long SEMAPHORE_N1_INTR;
extern unsigned long VID_CVBS_INTR;
extern unsigned long NF_INTR;
extern unsigned long USB1_DMA_INTR;
extern unsigned long USB1_MC_INTR;
extern unsigned long USB0_DMA_INTR;
extern unsigned long USB0_MC_INTR;
extern unsigned long UART2_INTR;
extern unsigned long UART3_INTR;
extern unsigned long VID_HDMI_INTR;
extern unsigned long VE_INTR;
extern unsigned long POK_OUT_INTR;
extern unsigned long SEMAPHORE_N0_INTR;
extern unsigned long HRX_HOT_PLUG_INTR;
extern unsigned long SCPU_ILB_ACK_INTR;
extern unsigned long SCPU_BUS_ERR_INTR;

extern unsigned long MBOX_INTR;
extern unsigned long __STANDBY_TEXT_START;
extern unsigned long __STANDBY_TEXT_END;
extern unsigned long __STANDBY_DATA_START;
extern unsigned long __STANDBY_DATA_END;
extern unsigned long __STANDBY_BSS_START;
extern unsigned long __STANDBY_BSS_END;

extern unsigned long _cpustats_start;
extern unsigned long _cpustats_end;

#endif