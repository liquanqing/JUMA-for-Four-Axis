/**
  ******************************************************************************
  * @file    Templates/Src/main.c
  * @author  MCD Application Team
  * @version V1.2.3
  * @date    29-January-2016
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "I2CSoft.h"
#include "HTS221.h"
#include "lsm6ds3.h"
#include "lsm303agr.h"
#include "stm324xg_eval_sd.h"
#include "ff_gen_drv.h"
#include "sd_diskio.h"
/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup Templates
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/


char SDpath[4]; /*SD card logical drives paths */
unsigned char write_buffer[512];                                         //写文件缓冲区
unsigned char read_buffer[512];     
FIL file;																 //文件对象
FATFS fatfs;															 //逻辑驱动器的工作区

int32_t Data[3] = {0};
int32_t Gata[3] = {0};
int32_t _Magnetic_mGa[3] = {0};
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);


/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */


int main(void)
{
    uint32_t i;
    unsigned int counter;
	static FRESULT res;						 //FRESULT函数公共结果代码
    DIR dir;
	FILINFO FileInfo;
    //---------------
    uint8_t id;
    float Hum;
    float Temp;
    
    IMU_6AXES_InitTypeDef InitStructure;
    
    /* Configure sensor */
    InitStructure.G_FullScale      = 2000.0f; /* 2000DPS */
    InitStructure.G_OutputDataRate = 200.0f;  /* 104HZ */
    InitStructure.G_X_Axis         = 1;       /* Enable */
    InitStructure.G_Y_Axis         = 1;       /* Enable */
    InitStructure.G_Z_Axis         = 1;       /* Enable */

    InitStructure.X_FullScale      = 2.0f;    /* 2G */
    InitStructure.X_OutputDataRate = 200.0f;  /* 104HZ */
    InitStructure.X_X_Axis         = 1;       /* Enable */
    InitStructure.X_Y_Axis         = 1;       /* Enable */
    InitStructure.X_Z_Axis         = 1;       /* Enable */
    /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, Flash preread and Buffer caches
       - Systick timer is configured by default as source of time base, but user
             can eventually implement his proper time base source (a general purpose
             timer for example or other time source), keeping in mind that Time base
             duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
             handled in milliseconds basis.
       - Low Level Initialization
     */
    HAL_Init();

    /* Configure the System clock to have a frequency of 84 MHz */
    SystemClock_Config();

    /* Add your application code here
     */
    //UsbDeviceInit();
    HAL_Delay(3000);
    
    i2c_soft.dev_on();
    
    while(IMU_6AXES_OK != LSM6DS3Drv.Init(&InitStructure)){
        HAL_Delay(200);
    }
    
    if(IMU_6AXES_OK == LSM6DS3Drv.Read_XG_ID(&id)){
        while( id != I_AM_LSM6DS3_XG){
          
        }
    }

    while(HUM_TEMP_SUCCESS != hts221.dev_on(&i2c_soft)){
        HAL_Delay(200);
    }
    if(HUM_TEMP_SUCCESS == hts221.read_id(&id)){
        while( id != I_AM_HTS221){

        }
    }
    
    while(!init_LSM303AGR_mag(LSM303AGR_MAG_ODR_100Hz))
    {

    }

    
    //---------fatfs-----------------------------------------------------
    if(FATFS_LinkDriver(&SD_Driver, SDpath) == 0){
        res = f_mount(&fatfs,(TCHAR const*)SDpath,0);                       //挂载SD卡
        if(res != RES_OK){
            while(1){
                printf("f_mount error!\r\n");
                HAL_Delay(1000);
                
            }
        }else{
            printf("f_mount successful!\r\n");
            
        }
    }
    
    if(FR_OK == f_opendir(&dir, "0:/")){
        while(FR_OK == f_readdir(&dir, &FileInfo)){
            if(!FileInfo.fname[0])break;

            if(FileInfo.fattrib  == AM_ARC)
                printf("文件名:%s\r\n",FileInfo.fname);
        }
    }
    
    res = f_open(&file,"0:/test.txt",FA_READ | FA_WRITE | FA_OPEN_ALWAYS);   //打开驱动器0上的源文件
    if(res != RES_OK){

        while(1){
            HAL_Delay(1000);
            printf("f_open error!\r\n");
        }
    }else{
        printf("f_open successful!\r\n");
    }
    
	res = f_lseek(&file,0);                                                  //移动写指针到文件首
	if(res != RES_OK){

		while(1){
			HAL_Delay(1000);
			printf("f_lseek error!\r\n");
		}
	}else{
		printf("f_lseek successful!\r\n");
	}

	for(i = 0;i < 512;i++){
		write_buffer[i] = i % 256;
	}
	res = f_write(&file,"你好啊",strlen("你好啊"),&counter);                          //将缓冲器中的内容写入源文件
	if(res != RES_OK || counter != strlen("你好啊")){
		while(1){
			HAL_Delay(1000);
			printf("f_write error!\r\n");
		}
	}else{
		printf("f_write successful!\r\n");
	}

	res = f_lseek(&file,0);	                                                 //移动读指针到文件首
	if(res != RES_OK){
		while(1){
			HAL_Delay(1000);
			printf("f_lseek error!\r\n");
		}
	}else{
		printf("f_lseek successful!\r\n");
	}

	res = f_read(&file,read_buffer,512,&counter);
	if(res != RES_OK){
		while(1){
			HAL_Delay(1000);
			printf("f_read error!\r\n");
		}
	}else{
		printf("f_read successful!\r\n");
	}
	f_close(&file);                                                          //关闭源文件


	printf("read data:\r\n");
	for(i = 0;i < counter;i++){
		printf(" %02X",read_buffer[i]);
        printf("\r\n");
        if(0 == i%16 && i != 0){
            printf("\r\n");
        }
	}

    /* Infinite loop */
    while (1){
        //i2c_multi_read(0xBE,0x0F,&id, 1);
        //i2c_soft.multi_read_byte(0xBE,0x0F,&id, 1);
        //while(CDC_Transmit_FS(&id,sizeof(id)));
        //Main_loop();
        hts221.get_humidity(&Hum);
        hts221.get_temperature(&Temp);
        LSM6DS3Drv.Get_X_Axes(Data);
        LSM6DS3Drv.Get_G_Axes(Gata);
        LSM303AGR_MAG_Get_Magnetic(_Magnetic_mGa);
        
        printf("humidity = %f\r\n",Hum);
        printf("temperature = %f\r\n",Temp);
        
        printf("Magnetic_MAG.X = %d\r\n",_Magnetic_mGa[0]);
        printf("Magnetic_MAG.Y = %d\r\n",_Magnetic_mGa[1]);
        printf("Magnetic_MAG.Z = %d\r\n",_Magnetic_mGa[2]);
        
        printf("LSM6DS3Drv_ACC.X = %d\r\n",Data[0]);
        printf("LSM6DS3Drv_ACC.Y = %d\r\n",Data[1]);
        printf("LSM6DS3Drv_ACC.Z = %d\r\n",Data[2]);
        
        printf("LSM6DS3Drv_GRY.X = %d\r\n",Gata[0]);
        printf("LSM6DS3Drv_GRY.Y = %d\r\n",Gata[1]);
        printf("LSM6DS3Drv_GRY.Z = %d\r\n",Gata[2]);
        HAL_Delay(2000);
    }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 84000000
  *            HCLK(Hz)                       = 84000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 336
  *            PLL_P                          = 4
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale2 mode
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{

    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;

    __PWR_CLK_ENABLE();

    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

}


#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
