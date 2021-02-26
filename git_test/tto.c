#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "prt_error_code.h"
#include "driver_libopt.h"
#include "prt_driver.h"
#include "prt_pwm.h"
#include "prt_spi_device.h"
#include "prt_gpio.h"
#include "prt_pwm_gd32e50x.h"
#include "lcd_driver.h"
#include "log_manager.h"
#include "prt_gpio_gd32e50x_prt_rdt6.h"
#include "prt_spi_gd32e50x.h"
#include "prt_interrupt_gd32e50x.h"


#include "gd32e50x_rcu.h"
#include "Prt_os_task.h"
#include "Prt_os_sleep.h"
#include "Prt_sqpi_psram_gd32eprt.h"
#include "prt_malloc.h"
#include "prt_flash_device.h"
#include "prt_flash_device_spi.h"
#include "prt_partition_new.h"
#include "prt_partition_flash_new.h"

#include "keyboard.h"

#include "prt_data_com.h"

#define DRIVER_DEBUG

#include "prt_debug.h"

#include "prt_uart.h"
#include "prt_uart_gd32e50x.h"
#include "srv_pl.h"

#include "GUI_LCD.h"
#include "protocol.h"
#include "prt_by_malloc.h"
#include "command.h"
#include "GUI.h"
#include "log_manager.h"

#include "prt_menu.h"


void recv_proc(void);


static prt_device_controller_info_t* prt_uart_controller_list[1];
static const prt_uart_gd32e50x_userdata_t uart_gd32e50x_userdata[] = {
    {
        .rts_gpio = PRT_GPIO_INACION,
        .cts_gpio = PRT_GPIO_INACION,
        .rx_size = 256,
        .bus_num = 1,
        .data_mode = 0,
        .dma_buffer_size = 128,
    },

    {
        .rts_gpio = PRT_GPIO_INACION,//PRT_GPIO_INACION,
        .cts_gpio = PRT_GPIO_INACION,
        .rx_size = 1250,
        .data_mode = 0,
        .dma_buffer_size = 128,
        .bus_num = 2,
        //.flow_on_threshold = 600,
        //.flow_off_threshold = 700,
    },

};
static const prt_device_controller_info_t uart_gd32e50x_controller_info = {
    .id_start = 1,
    .id_end = ARRAY_SIZE(uart_gd32e50x_userdata),
    .pdata = (void*)uart_gd32e50x_userdata,
    .open = prt_uart_gd32e50x_open,
};

static prt_device_controller_info_t* prt_flash_spi_device_controller_list;
static const prt_flash_spi_userdata_t flash_spi_user_data = {
    .idx = 0,
    .spi_id = 1,
};
const static prt_device_controller_info_t prt_flash_spi_controller_info = {
	.id_start = 0,
	.id_end = 0,
	.pdata = &flash_spi_user_data,
	.open = prt_flash_spi_open,
};

static prt_device_controller_info_t* prt_partition_controller_list;
static prt_partition_uaserdata_t partition_uaserdata[] = {
    {.idx = 0, .storage_id = 0,  .name = (uint8_t *)"firmware",     .type = PRT_PARTITION_TYPE_BOOT,      .size = 300*1024},
    {.idx = 1, .storage_id = 0,  .name = (uint8_t *)"res",          .type = PRT_PARTITION_TYPE_FIRMWARE,  .size = (3*1024+700)*1024},	
	/*************************************************************---------------------------------------------------------------------



	-----------------------------------------------------------------------------------------------------------------------------------*/
    {.idx = 2, .storage_id = 0,  .name = (uint8_t *)"flash",          .type = PRT_PARTITION_TYPE_FIRMWARE,  .size = 100},
};
const static prt_device_controller_info_t partition_flash_controller_info = {
    .id_start = 0,
    .id_end = 2,
    .pdata = partition_uaserdata,
    .open = prt_partition_flash_open,
};

static prt_device_controller_info_t* prt_gpio_controller_list;
static const prt_device_controller_info_t gpio_gd32e50x_controller_info = {
    .id_start = 1,
    .id_end = 64,
    .open = prt_gpio_gd32e50x_open,
};

static prt_device_controller_info_t* prt_spi_controller_list;
static const prt_spi_gd32e50x_userdata_t spi_gd32e50x_userdata[] = {
    {
        .bus = 0,
        .freq = 9000000,
        .gpio_cs_num = 20,
        .master_or_slave = PRT_SPI_MASTER,
        .first_bit = PRT_SPI_MSB_FIRST,
        .data_size = PRT_SPI_DATA_SIZE_8,
        .mode = PRT_SPI_MODE0,
    },
    {
        .bus = 1,
        .freq = 22000000,
        .gpio_cs_num = PRT_GPIO_INACION,
        .master_or_slave = PRT_SPI_MASTER,
        //.bus_mode =  PRT_SPI_TX_DMA,
        .first_bit = PRT_SPI_MSB_FIRST,
        .data_size = PRT_SPI_DATA_SIZE_8,
        .mode = PRT_SPI_MODE0,
    },
    {
        .bus = 2,
        .freq = 6000000,
        .gpio_cs_num = PRT_GPIO_INACION,
        .master_or_slave = PRT_SPI_MASTER,
       // .bus_mode = PRT_SPI_TX_DMA,
        .first_bit = PRT_SPI_MSB_FIRST,
        .data_size = PRT_SPI_DATA_SIZE_8,
        .mode = PRT_SPI_MODE0,
    },
};
static const prt_device_controller_info_t spi_gd32e50x_controller_info = {
    .id_start = 1,
    .id_end = 3,
    .pdata = (void*)spi_gd32e50x_userdata,
    .open = prt_spi_gd32e50x_open,
};

static prt_device_controller_info_t* prt_pwm_controller_list;
static const prt_pwm_gd32e50x_userdata_t pwm_gd32e50x_userdata[] = {
     {
     .timer = 2,
     .remap_mode = 0,
     .channel = 2,
     .idle_is_high = 0,},   //tone
     {
     .timer = 0,
     .remap_mode = 0,
     .channel = 0,
     .idle_is_high = 0,
     },
};
static const prt_device_controller_info_t  pwm_gd32e50x_controller_info = {
    .id_start = 1,
    .id_end = ARRAY_SIZE(pwm_gd32e50x_userdata),
    .pdata = &pwm_gd32e50x_userdata,
    .open = prt_pwm_gd32e50x_open,
};

int fputc(int tempch, FILE *f)
{
    usart_data_transmit(USART0, (uint8_t)tempch);
    while(RESET == usart_flag_get(USART0, USART_FLAG_TBE));
    return tempch;
}

static uint32_t global_heap[1024*32/4]; //32K
static prt_mem_t heap_info;
static prt_mem_t sram_info;

extern uint8_t ucHeap[];

static result_t prt_mem_reg (void)
{
	heap_info.type = PRT_MEM_TYPE_IRAM;
	heap_info.capacity = sizeof(global_heap);
	heap_info.address = (uint32_t)&global_heap;
	prt_mem_malloc_add(&heap_info);
    SYS_INFO("heap_info:%x\n",(unsigned int)global_heap);
    SYS_INFO("ucHeap:%x\n",(unsigned int)ucHeap);

	sram_info.type = PRT_MEM_TYPE_SRAM;
	sram_info.capacity =4*1024*1024 - (2*1024*1024);
	sram_info.address  = SQPI_LOGIC_ADDR + (2*1024*1024);
	prt_mem_malloc_add(&sram_info); // 分配堆
    return E_OK;
}

 void prt_debug_uart_init(void)//for debug
{
    /* enable GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOA);

    /* enable USART clock */
    rcu_periph_clock_enable(RCU_USART0);

    /* configure the USART0 Tx pin and USART0 Rx pin */
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_10);

     /* USART configure */
    usart_deinit(USART0);
    usart_baudrate_set(USART0, 115200U);
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
    usart_enable(USART0);

}

static result_t prt_hardware_resources_init()
{
    prt_interrupt_gd32e50x_init();
    prt_uart_controller_list[0] = (prt_device_controller_info_t*)&uart_gd32e50x_controller_info;
    prt_uart_add_controller ((const prt_device_controller_info_t **)prt_uart_controller_list, 1);
	prt_gpio_controller_list = (prt_device_controller_info_t*)&gpio_gd32e50x_controller_info;
	prt_gpio_add_controller((const prt_device_controller_info_t **)&prt_gpio_controller_list, 1);
	prt_spi_controller_list = (prt_device_controller_info_t*)&spi_gd32e50x_controller_info;
	prt_spi_device_add_controller((const prt_device_controller_info_t **)&prt_spi_controller_list, 1);
	prt_pwm_controller_list = (prt_device_controller_info_t*)&pwm_gd32e50x_controller_info;
	prt_pwm_add_controller ((const prt_device_controller_info_t **)&prt_pwm_controller_list, 1);
	prt_flash_spi_device_controller_list = (prt_device_controller_info_t*)&prt_flash_spi_controller_info;
	prt_flash_device_add_controller((const prt_device_controller_info_t **)&prt_flash_spi_device_controller_list, 1);
	prt_partition_controller_list = (prt_device_controller_info_t*)&partition_flash_controller_info;
	prt_partition_add_controller((const prt_device_controller_info_t **)&prt_partition_controller_list, 2);

	return E_OK;
}

void mytest(void)
{
	int i,j = 0;
	uint16_t colors[] = {0xf800,0xffe0,0x07e0,0x07ff,0x001f,0xf81f};
	lcd_display_color(0xffff);
	while(1)
	{
		lcd_display_str(100,50,145,70,(uint8_t *)"hello",5,0,colors[j],0x0);
		j++;
		j = j % 6;
		i = 5000000;
		while(i-- > 0);
	}
}

#include "srv_lcd.h"
int  system_sqpi_psram_init(void);
void init_uart(void)
{
    prt_dev_hand_t port;
    prt_gpio_config_t cfg = {.dir = PRT_GPIO_OUT, .mode = PRT_GPIO_NORMAL};
    prt_uart_cfg_t cfg_uart;

    port = prt_uart_open(1);//uart1
    prt_assert(port);
    

    memset((uint8_t *)&cfg_uart, 0x00, sizeof(cfg_uart));

    cfg_uart.baudrate = 115200;
    cfg_uart.flow_mode = 0xFF;
    cfg_uart.parity = PRT_UART_PARITY_NONE;
    cfg_uart.stop_bits = PRT_UART_STOPBITS_1;
    cfg_uart.word_length = 8;
    cfg_uart.int_mode = PRT_UART_RX_IT_ENABLE;
    prt_uart_config(port,&cfg_uart);

    prt_uart_cmd(port,PRT_FUN_ENABLE);
    prt_data_com_port_add(port,"uart1");

    port = prt_gpio_open(15);
    prt_gpio_config (port, &cfg);
    prt_gpio_set(port);
    
    

}
static prt_dev_hand_t prt_flash_res;

static void comm_deal_thread(void * pvParameters )
{
    static uint8_t buf[100];


    prt_dev_hand_t prt_flash= prt_flash_device_open(0);
    prt_assert(prt_flash);

    if(prt_flash_device_read(prt_flash,0,buf,20)==E_OK)
    {
        SYS_INFO("prt_flash_device_read OK\n");
    }else
    {
        SYS_ERR("prt_flash_device_read error\n");
    }


  while (1)
   {
        //if(setting_command_analy())
        //{
            recv_proc();
        //}
   }

}

/* 读取 GUI 资源数据 */
U32 GUI_ResReadData(U32 offset, U8* buf, U32 len)
{
  if (prt_partition_read(prt_flash_res, offset, buf, len) == E_OK){
    return len;
  }
  return 0;
}
extern int log_to_linux_enable;

void init_comm(void)
{
    prt_data_com_task_init();
    init_uart();
    pro_init();
    log_to_linux_enable = 1;
    xTaskCreate(comm_deal_thread, "comm", 786, NULL, 5, ( TaskHandle_t * ) NULL );
}
int main(void)
{
    prt_dev_hand_t port;
    
	result_t ret;
	
	__disable_irq();
	prt_debug_uart_init();
	printf("\r\ndebug init ok\r\n");
    printf("Date:%s\n", __DATE__);
    printf("Time:%s\n", __TIME__);
	nvic_priority_group_set(NVIC_PRIGROUP_PRE4_SUB0);
	//prt_sqpi_psram_init();
	printf("CK_SYS:%d\n", rcu_clock_freq_get(CK_SYS));
    printf("CK_AHB:%d\n", rcu_clock_freq_get(CK_AHB));
    printf("CK_APB1:%d\n", rcu_clock_freq_get(CK_APB1));
    printf("CK_APB2:%d\n", rcu_clock_freq_get(CK_APB2));
    
	prt_mem_reg();
	prt_hardware_resources_init();
       log_init();
	hy_mem_init();
    	init_comm();

	cmd_init();
    SYS_INFO("Date:%s\n", __DATE__);
    SYS_INFO("Time:%s\n", __TIME__);
    
	SYS_INFO("to init lcd !!\r\n");
	ret = lcd_init();
	if(E_OK != ret)
	{
		SYS_ERR("lcd_init err\r\n");
	}
	//mytest();
	KeyBoardValeInit();
    prt_flash_res = prt_partition_open_by_name((uint8_t*)"res");
    prt_assert(prt_flash_res);
    static uint8_t buf[100];
    GUI_ResReadData(0,buf,100);
    if (memcmp(&buf[0x0c],"resource",8)!=0)//字库不存在
    {
        SYS_ERR("font lib not found\r\n");
        GUI_Init();
        GUI_SetBkColor(GUI_DARKGRAY);
        GUI_SetColor(GUI_RED);
        GUI_Clear();
        GUI_SetFont(&GUI_Font24B_1);
        GUI_DispStringHCenterAt("Please download font lib", 160, 80);
        GUI_Delay(1000);
    }else
    {  
    	srv_lcd_init();
    }
	lcd_display_color(0xFFFF);
	//lcd_display_color(0XFFAAFF);
	SYS_INFO("os start!\r\n");
	vTaskStartScheduler();
	while(1);
}
void prt_set_into_boot_flag(void);

void Hardfault_debug(unsigned int * hardfault_args, int psp)
{
    unsigned int stacked_r0;
    unsigned int stacked_r1;
    unsigned int stacked_r2;
    unsigned int stacked_r3;
    unsigned int stacked_r12;
    unsigned int stacked_lr;
    unsigned int stacked_pc;
    unsigned int stacked_psr;
    prt_debug_uart_init();
    //prt_stop_print();
    stacked_r0 = ((unsigned long) hardfault_args[0]);

    stacked_r1 = ((unsigned long) hardfault_args[1]);

    stacked_r2 = ((unsigned long) hardfault_args[2]);

    stacked_r3 = ((unsigned long) hardfault_args[3]);



    stacked_r12 = ((unsigned long) hardfault_args[4]);
    stacked_lr = ((unsigned long) hardfault_args[5]);

    stacked_pc = ((unsigned long) hardfault_args[6]);

    stacked_psr = ((unsigned long) hardfault_args[7]);
    printf ("\r\n\r\n\r\n[APP][Hard fault handler]\n");

    printf ("R0 = %x\r\n", stacked_r0);

    printf ("R1 = %x\r\n", stacked_r1);

    printf ("R2 = %x\r\n", stacked_r2);

    printf ("R3 = %x\r\n", stacked_r3);

    printf ("R12 = %x\r\n", stacked_r12);

    printf ("LR = %x\r\n", stacked_lr);

    printf ("PC = %x\r\n", stacked_pc);

    printf ("PSR = %x\r\n", stacked_psr);
#if 0
    printf ("BFAR = %lx\n", (*((volatile unsigned long *)(0xE000ED38))));

    printf ("CFSR = %lx\n", (*((volatile unsigned long *)(0xE000ED28))));

    printf ("HFSR = %lx\n", (*((volatile unsigned long *)(0xE000ED2C))));

    printf ("DFSR = %lx\n", (*((volatile unsigned long *)(0xE000ED30))));

    printf ("AFSR = %lx\n", (*((volatile unsigned long *)(0xE000ED3C))));

    printf("MSP = %x\n", (uint32_t)hardfault_args);
    printf("PSP = %x\n", psp);
#endif
    prt_set_into_boot_flag();
    while (1) {
        NVIC_SystemReset();
    }
}

#if (configUSE_MALLOC_FAILED_HOOK == 1)
void vApplicationMallocFailedHook( void )
{
	printf("malloc fail\n");
	printf("task:%s overflow\n",pcTaskGetName(NULL));
  configASSERT(0);
}
#endif

#if ( configCHECK_FOR_STACK_OVERFLOW > 1 )
//extern PRIVILEGED_DATA TCB_t * volatile pxCurrentTCB = NULL;
void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;
	printf("task:%s overflow\n",pcTaskGetName(NULL));
    configASSERT(0);
}
#endif

void vApplicationIdleHook( void )
{

}
