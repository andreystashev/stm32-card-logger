/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "sd.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "time.h"
#include <stdbool.h>
#include <unistd.h>



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_SPEED 460800
#define BUFSIZE 420//есть предположение что прерывание срабатывает каждые 42 байта, увеличил в 20 для проверки(предположительно не влиеяет)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
RTC_TimeTypeDef sTime = {0};
RTC_DateTypeDef DateToUpdate = {0};
char trans_str[64] = {0,};
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



uint8_t rx_buff[BUFSIZE] = {0,};
uint16_t rx_buff_len;
volatile uint16_t Timer1=0;
uint8_t sect[512];
extern char str1[60];
uint32_t byteswritten,bytesread;
uint8_t result;
uint8_t newdir;
extern char USERPath[4]; /* logical drive path */

FATFS SDFatFs;
FATFS *fs;
FIL MyFile;

//uint8_t buffer[size];

FRESULT res;
FILINFO fileInfo;
char *fn;
DIR dir;
DWORD fre_clust, fre_sect, tot_sect;

uint8_t fullMessageBuffer[800] = {0,};//массив большой для всех сообщений с запасом. был 400 увеличил в 2 раза для проверки
uint8_t *pfullMessageBuffer = fullMessageBuffer;
uint16_t fullMessageLength = 0; // сюда длину сообщения из заголовка пишем
int headerSwitcher = 1; // переключатель чтения заголовка/тела пакета
int lighterSwitcher = 1; // переключатель светодиода
uint16_t addressMessage = 0; // тут адрес пишем и смотрим бит для пр
uint8_t statusMessage[8]={0x3A,0x01,0x00,0x07,0x00,0x00,0x40,0x89};//массив с положительным статусом
uint8_t nothingToSendMessage[8]={0x3A,0x01,0x00,0x01,0x00,0x00,0x41,0x69};//массив с положительным статусом
int rewriteCounterCLoseFile = 0;

uint8_t rewriteMessagesBuffer[3400] = {0,};//massive for rewrite many messages
uint16_t rewriteMessagesCounter = 0;


//считыванием содержимого нашего файла
FRESULT ReadLongFile(void)
{
    uint16_t i=0, i1=0;
    uint32_t ind=0;
    uint32_t f_size = MyFile.fsize;
    sprintf(str1,"fsize: %lu\r\n",(unsigned long)f_size);
    HAL_UART_Transmit(&huart2,(uint8_t*)str1,strlen(str1),0x1000);
    ind=0;
    do
    {
        if(f_size<512)
        {
            i1=f_size;
        }
        else
        {
            i1=512;
        }
        f_size-=i1;
        f_lseek(&MyFile,ind);
        f_read(&MyFile,sect,i1,(UINT *)&bytesread);
        for(i=0;i<bytesread;i++)
        {
            HAL_UART_Transmit(&huart2,sect+i,1,0x1000);
        }
        ind+=i1;
    }
    
    while(f_size>0);
    HAL_UART_Transmit(&huart1,(uint8_t*)"\r\n",2,0x1000);
    return FR_OK;
    
}

//THREAD AI



//THREAD END




void rewrite(const void* buff,int size)
{
    //write
    
    
    if(f_mount(&SDFatFs,(TCHAR const*)USERPath,0)!=FR_OK)
    {
        Error_Handler();
    }
    else
    {
        if(f_open(&MyFile,"mywrite.txt",FA_OPEN_ALWAYS|FA_WRITE)!=FR_OK)//FA_OPEN_ALWAYS//FA_CREATE_ALWAYS
        {
            Error_Handler();
        }
        else
        {
            rewriteMessagesCounter = 0;
            //	    	uint8_t t[8]="Hello";
            //			res = f_write(&MyFile,t,sizeof(t),(void*)&byteswritten);
            //	    	res = f_lseek(&MyFile, f_size(&MyFile));
            //	    	res = f_write(&MyFile,"\r\n",2,(void*)&byteswritten);
            res = f_lseek(&MyFile, f_size(&MyFile));
            
            res = f_write(&MyFile,buff,size,(void*)&byteswritten);
            //			HAL_UART_Transmit(&huart3 , "WRITE", 6,10 );
            
            //	    	res = f_write(&MyFile,t,sizeof(t),(void*)&byteswritten);
            
            if((byteswritten==0)||(res!=FR_OK))
            {
                Error_Handler();
            }
            //закрываем файл не всегда а спустя 20 записей переключаем светодиод
            //	      rewriteCounterCLoseFile+=1;
            //	      if(rewriteCounterCLoseFile==6){
            //	    	  rewriteCounterCLoseFile=0;
            if(lighterSwitcher==1)
            {
                HAL_GPIO_WritePin(parachute_GPIO_Port, parachute_Pin, GPIO_PIN_SET);
                lighterSwitcher=0;
            }
            else if(lighterSwitcher==0)
            {
                HAL_GPIO_WritePin(parachute_GPIO_Port, parachute_Pin, GPIO_PIN_RESET);
                lighterSwitcher=1;
            }
            
            f_close(&MyFile);
            
            //	      }
            //	      f_close(&MyFile);
            
        }
    }
}

void CreateNewFile(const TCHAR* filename, const void* mass)
{
    if(f_mount(&SDFatFs,(TCHAR const*)USERPath,0)!=FR_OK)
    {
        Error_Handler();
    }
    else
    {
        if(f_open(&MyFile,filename,FA_CREATE_ALWAYS|FA_WRITE)!=FR_OK)//FA_OPEN_ALWAYS
        {
            Error_Handler();
        }
        else
        {
            res=f_write(&MyFile,mass,sizeof(mass),(void*)&byteswritten);
            
            if((byteswritten==0)||(res!=FR_OK))
                
            {
                Error_Handler();
            }
            f_close(&MyFile);
        }
    }
}

void ReadDir()
{
    
    if(f_mount(&SDFatFs,(TCHAR const*)USERPath,0)!=FR_OK)
    {
        Error_Handler();
    }
    else
    {
        fileInfo.lfname = (char*)sect;
        fileInfo.lfsize = sizeof(sect);
        result = f_opendir(&dir, "/");
        if (result == FR_OK)
        {
            while(1)
            {
                result = f_readdir(&dir, &fileInfo);
                
                if (result==FR_OK && fileInfo.fname[0])
                {
                    fn = fileInfo.lfname;
                    if(strlen(fn)) HAL_UART_Transmit(&huart2,(uint8_t*)fn,strlen(fn),0x1000);
                    else HAL_UART_Transmit(&huart2,(uint8_t*)fileInfo.fname,strlen((char*)fileInfo.fname),0x1000);
                    if(fileInfo.fattrib&AM_DIR)
                    {
                        HAL_UART_Transmit(&huart2,(uint8_t*)" [DIR]",7,0x1000);
                    }
                }
                else break;
                HAL_UART_Transmit(&huart2,(uint8_t*)"\r\n",2,0x1000);
            }
            //информация о карте start
            //	  f_getfree("/", &fre_clust, &fs);
            //	  sprintf(str1,"fre_clust: %lurn",fre_clust);
            //	  HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
            //	  sprintf(str1,"n_fatent: %lurn",fs->n_fatent);
            //	  HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
            //	  sprintf(str1,"fs_csize: %drn",fs->csize);
            //	  HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
            //	  tot_sect = (fs->n_fatent - 2) * fs->csize;
            //	  sprintf(str1,"tot_sect: %lurn",tot_sect);
            //	  HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
            //	  fre_sect = fre_clust * fs->csize;
            //	  sprintf(str1,"fre_sect: %lurn",fre_sect);
            //	  HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
            //	  sprintf(str1, "%lu KB total drive space.rn%lu KB available.rn",fre_sect/2, tot_sect/2);
            //	  HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
            //информация о карте finish
            f_closedir(&dir);
        }
    }
}

void send_file_over_uart2(const char* filename) {
    // Открываем файл
    if (f_mount(&SDFatFs, (TCHAR const*)USERPath, 0) != FR_OK) {
        // Обработка ошибки монтирования файловой системы
        char *error10 = "error mount";
        HAL_UART_Transmit(&huart2, (uint8_t *)error10, strlen(error10), 10);
    }
    FIL file;
    if (f_open(&file, filename, FA_READ) != FR_OK) {
        // Обработка ошибки открытия файла
        char *error1 = "file not open";
        HAL_UART_Transmit(&huart2, (uint8_t *)error1, strlen(error1), 10);
    }
    
    // Определяем размер файла
    UINT file_size = f_size(&file);
    
    // Отправляем размер файла по UART2
    char size_buffer[20];
    snprintf(size_buffer, sizeof(size_buffer), "%lu", file_size);
    HAL_UART_Transmit(&huart2, (uint8_t*)size_buffer, strlen(size_buffer), 10);
    
    // Передаем содержимое файла по UART2
    uint8_t buffer[1024];
    UINT bytes_read, bytes_sent;
    while (f_read(&file, buffer, 1024, &bytes_read) == FR_OK && bytes_read > 0) {
        if (HAL_UART_Transmit(&huart2, buffer, bytes_read, 30) != HAL_OK) {
            // Обработка ошибки передачи данных
            char *error2 = "HAL_UART_Transmit not transmit";
            HAL_UART_Transmit(&huart2, (uint8_t *)error2, strlen(error2), 10);
            break;
        }
        bytes_sent += bytes_read;
        //    snprintf(bytes_sent, sizeof(bytes_sent), "%lu", bytes_sent);
        
        //    printf("Sent %d bytes\r\n", bytes_sent); // Отладочный вывод
        char sent_buffer[32]; // Создаем буфер для хранения отформатированной строки
        snprintf(sent_buffer, sizeof(sent_buffer), "Sent %d bytes\r\n", bytes_sent); // Форматируем строку и сохраняем ее в буфер
        HAL_UART_Transmit(&huart2, (uint8_t*)sent_buffer, strlen(sent_buffer), 10);
        //    if (HAL_UART_Transmit(&huart2, (uint8_t *)sent_buffer, strlen(sent_buffer), HAL_MAX_DELAY) != HAL_OK) {
        //      // Обработка ошибки передачи данных
        //      break;
        //    }
    }
    
    // Закрываем файл
    f_close(&file);
    char *error5 = "all good! Data transmited";
    HAL_UART_Transmit(&huart2, (uint8_t *)error5, strlen(error5), 10);
}

void fileTransfer()
{
    char *error0 = "start file transmited";
    HAL_UART_Transmit(&huart2, (uint8_t *)error0, strlen(error0), 10);
    
    FILE *fp;
    long int size;
    
    fp = fopen("mywrite.txt", "rb"); // Открываем файл для чтения в двоичном режиме
    if (fp == NULL) {
        // Обработка ошибки открытия файла
        char *error1 = "mywrite.txt not open";
        HAL_UART_Transmit(&huart2, (uint8_t *)error1, strlen(error1), 10);
    }
    
    fseek(fp, 0L, SEEK_END); // Устанавливаем позицию в конец файла
    size = ftell(fp); // Получаем текущую позицию, что равно размеру файла
    fseek(fp, 0L, SEEK_SET); // Устанавливаем позицию в начало файла
    
    uint8_t buffer[1024];
    uint32_t bytes_sent = 0;
    uint32_t bytes_to_send = size;
    
    // Передаем размер файла
    if (HAL_UART_Transmit(&huart2, (uint8_t *)&size, sizeof(size), HAL_MAX_DELAY) != HAL_OK) {
        // Обработка ошибки передачи данных
        char *error2 = "HAL_UART_Transmit not transmit";
        HAL_UART_Transmit(&huart2, (uint8_t *)error2, strlen(error2), 10);
    }
    
    // Передаем содержимое файла по блокам размером BUFFER_SIZE байт
    while (bytes_sent < bytes_to_send) {
        uint32_t bytes_to_read = (bytes_to_send - bytes_sent) > 1024 ? 1024 : (bytes_to_send - bytes_sent);
        
        if (fread(buffer, 1, bytes_to_read, fp) != bytes_to_read) {
            // Обработка ошибки чтения файла
            char *error3 = "file not read";
            HAL_UART_Transmit(&huart2, (uint8_t *)error3, strlen(error3), 10);
        }
        
        if (HAL_UART_Transmit(&huart2, buffer, bytes_to_read, HAL_MAX_DELAY) != HAL_OK) {
            // Обработка ошибки передачи данных
            char *error4 = "data not transmit";
            HAL_UART_Transmit(&huart2, (uint8_t *)error4, strlen(error4), 10);
        }
        
        bytes_sent += bytes_to_read;
    }
    
    fclose(fp); // Закрываем файл
    char *error5 = "all good! Data transmited";
    HAL_UART_Transmit(&huart2, (uint8_t *)error5, strlen(error5), 10);
    
}

/* USER CODE END 0 */
uint8_t str[BUFSIZE+1];
uint8_t dataReceived=0; // признак данное получено
uint8_t dataTransmitted=1; // признак данное передано
/**
 * @brief  The application entry point.
 * @retval int
 */


void waitedRewrite()
{
    
    if(rewriteMessagesCounter>=500)//влияет на частоту вызова функции
    {
        if(lighterSwitcher==1)
        {
            HAL_GPIO_WritePin(parachute_GPIO_Port, parachute_Pin, GPIO_PIN_SET);
            lighterSwitcher=0;
        }
        else if(lighterSwitcher==0)
        {
            HAL_GPIO_WritePin(parachute_GPIO_Port, parachute_Pin, GPIO_PIN_RESET);
            lighterSwitcher=1;
        }
        rewrite(rewriteMessagesBuffer, rewriteMessagesCounter);
        
    }
}

int main(void)
{
    /* USER CODE BEGIN 1 */
    //	uint16_t i;
    //	FRESULT res; //результат выполнения
    //	uint8_t wtext[]="Hello from STM32!!!";
    
    
    /* USER CODE END 1 */
    
    /* MCU Configuration--------------------------------------------------------*/
    
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();
    
    /* USER CODE BEGIN Init */
    
    /* USER CODE END Init */
    
    /* Configure the system clock */
    SystemClock_Config();
    
    /* USER CODE BEGIN SysInit */
    
    /* USER CODE END SysInit */
    
    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_SPI2_Init();
    MX_TIM2_Init();
    MX_USART1_UART_Init();
    MX_FATFS_Init();
    MX_RTC_Init();
    MX_USART3_UART_Init();
    MX_USART2_UART_Init();
    /* USER CODE BEGIN 2 */
    HAL_TIM_Base_Start_IT(&htim2);
    
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    //SD_PowerOn();
    //sd_ini()
    //  HAL_UART_Transmit(&huart3 , statusMessage, 8,10 );//отправляем статус ок
    //  HAL_UART_Transmit(&huart2 , statusMessage, 8,10 );//отправляем статус ок
    //  HAL_UART_Transmit(&huart1 , statusMessage, 8,10 );//отправляем статус ок
    
    disk_initialize(SDFatFs.drv);
    
    //  ReadDir();
    //  send_file_over_uart2("mywrite.txt");
    //  fileTransfer();
    //  CreateNewFile("123456.txt", 46);
    
    if(f_mount(&SDFatFs,(TCHAR const*)USERPath,0)!=FR_OK)
    {
        Error_Handler();
    }
    else
    {
        if(f_open(&MyFile,"mywrite.txt",FA_READ)!=FR_OK)
        {
            Error_Handler();
        }
        else
        {
            //      ReadLongFile();//read file in start
            f_close(&MyFile);
        }
    }
    
    
    headerSwitcher = 1;
    lighterSwitcher = 1;
    //  HAL_UART_Receive_IT (&huart2, fullMessageBuffer, 42);//приходят данные сюда записываются в массив стр на размер буфера
    
    HAL_UART_Receive_IT (&huart2, pfullMessageBuffer, 6);//приходят данные сюда записываются в массив стр на размер буфера
    
    
    
    /* USER CODE END 2 */
    
    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        //	  HAL_UART_Transmit(&huart2 , statusMessage, 8,10 );//отправляем статус ок
        //	  HAL_Delay(700);
        //	  HAL_GPIO_WritePin(parachute_GPIO_Port, parachute_Pin, GPIO_PIN_SET);
        //	  char *message34 = "uart2 work";
        //	  HAL_UART_Transmit(&huart2, (uint8_t *)message34, strlen(message34), 10);
        //	  char *message345 = "uart1 work";
        //	  HAL_UART_Transmit(&huart1, (uint8_t *)message345, strlen(message345), 10);
        //	  char *message35 = "uart3 work";
        //	  HAL_UART_Transmit(&huart3, (uint8_t *)message35, strlen(message35), 10);
        
        //	  HAL_Delay(700);
        //	  HAL_GPIO_WritePin(parachute_GPIO_Port, parachute_Pin, GPIO_PIN_RESET);
        
        //	  waitedRewrite();
        //	  waitedRewrite();
        
        //	  HAL_Delay(300);//влияет на внезапные остановки на больших скоростях
        
        /* USER CODE END WHILE */
        
        /* USER CODE BEGIN 3 */
        
    }
    /* USER CODE END 3 */
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    
    
    if(huart == &huart2)
    {
        if(headerSwitcher != 0)
        {
            headerSwitcher = 0;
            fullMessageLength = (pfullMessageBuffer[5]<< 8) | pfullMessageBuffer[4];
            HAL_UART_Receive_IT(&huart2, pfullMessageBuffer+6, fullMessageLength+2);
        }
        else
        {
            headerSwitcher = 1;
            addressMessage = (pfullMessageBuffer[2]<< 8) | pfullMessageBuffer[1];
            
            bool prBit = addressMessage & 0x20;
            if(prBit)
            {
                bool statusByte = pfullMessageBuffer[3]==0x06;
                bool retranslateByte = pfullMessageBuffer[3]==0x03;
                bool transmittedByte = pfullMessageBuffer[3]==0x05;
                
                
                if(statusByte)
                {
                    
                    HAL_UART_Transmit(&huart2 , statusMessage, 8,10);//отправляем статус ок
                    for(int z=0;z<fullMessageLength+2+6;++z)
                    {
                        rewriteMessagesBuffer[z+rewriteMessagesCounter]=fullMessageBuffer[z];
                    }
                    for(int z=0;z<8;++z)
                    {
                        rewriteMessagesBuffer[z+rewriteMessagesCounter]=statusMessage[z];
                    }
                    rewriteMessagesCounter+=fullMessageLength+2+6+8;
                    
                    // waitedRewrite();
                    //				  rewrite(statusMessage, 8);
                    //				  rewrite(fullMessageBuffer, fullMessageLength+2+6);
                }
                else if(retranslateByte)
                {
                    HAL_UART_Transmit(&huart2 , pfullMessageBuffer, fullMessageLength+2+6,10);
                    //				  HAL_UART_Transmit_DMA(&huart2 , fullMessageBuffer, fullMessageLength+2+6);
                    
                    for(int z=0;z<fullMessageLength+2+6;++z)
                    {
                        rewriteMessagesBuffer[z+rewriteMessagesCounter]=fullMessageBuffer[z];
                    }
                    rewriteMessagesCounter+=fullMessageLength+2+6;
                    //  waitedRewrite();
                    
                    
                    //				  rewrite(fullMessageBuffer, fullMessageLength+2+6);
                }
                else if(transmittedByte)
                {
                    
                    HAL_UART_Transmit(&huart2 , nothingToSendMessage, 8,1);//отправляем статус ок
                    
                    for(int z=0;z<fullMessageLength+2+6;++z)
                    {
                        rewriteMessagesBuffer[z+rewriteMessagesCounter]=fullMessageBuffer[z];
                    }
                    for(int z=0;z<8;++z)
                    {
                        rewriteMessagesBuffer[z+rewriteMessagesCounter]=nothingToSendMessage[z];
                    }
                    rewriteMessagesCounter+=fullMessageLength+2+6+8;
                    //  waitedRewrite();
                    //				  rewrite(nothingToSendMessage, 8);
                    //				  rewrite(fullMessageBuffer, fullMessageLength+2+6);
                }
            }
            fullMessageLength = 0;
            HAL_UART_Receive_IT(&huart2, fullMessageBuffer, 6);
        }
    }
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    
    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    
    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
    |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief RTC Initialization Function
 * @param None
 * @retval None
 */
static void MX_RTC_Init(void)
{
    
    /* USER CODE BEGIN RTC_Init 0 */
    
    /* USER CODE END RTC_Init 0 */
    
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef DateToUpdate = {0};
    
    /* USER CODE BEGIN RTC_Init 1 */
    
    /* USER CODE END RTC_Init 1 */
    
    /** Initialize RTC Only
     */
    hrtc.Instance = RTC;
    hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
    hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
    if (HAL_RTC_Init(&hrtc) != HAL_OK)
    {
        Error_Handler();
    }
    
    /* USER CODE BEGIN Check_RTC_BKUP */
    
    /* USER CODE END Check_RTC_BKUP */
    
    /** Initialize RTC and set the Time and Date
     */
    sTime.Hours = 0x0;
    sTime.Minutes = 0x0;
    sTime.Seconds = 0x0;
    
    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
    {
        Error_Handler();
    }
    DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
    DateToUpdate.Month = RTC_MONTH_JANUARY;
    DateToUpdate.Date = 0x1;
    DateToUpdate.Year = 0x0;
    
    if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN RTC_Init 2 */
    
    /* USER CODE END RTC_Init 2 */
    
}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void)
{
    
    /* USER CODE BEGIN SPI2_Init 0 */
    
    /* USER CODE END SPI2_Init 0 */
    
    /* USER CODE BEGIN SPI2_Init 1 */
    
    /* USER CODE END SPI2_Init 1 */
    /* SPI2 parameter configuration*/
    hspi2.Instance = SPI2;
    hspi2.Init.Mode = SPI_MODE_MASTER;
    hspi2.Init.Direction = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi2.Init.NSS = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi2) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN SPI2_Init 2 */
    
    /* USER CODE END SPI2_Init 2 */
    
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{
    
    /* USER CODE BEGIN TIM2_Init 0 */
    
    /* USER CODE END TIM2_Init 0 */
    
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    
    /* USER CODE BEGIN TIM2_Init 1 */
    
    /* USER CODE END TIM2_Init 1 */
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 39999;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 10;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM2_Init 2 */
    
    /* USER CODE END TIM2_Init 2 */
    
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{
    
    /* USER CODE BEGIN USART1_Init 0 */
    
    /* USER CODE END USART1_Init 0 */
    
    /* USER CODE BEGIN USART1_Init 1 */
    
    /* USER CODE END USART1_Init 1 */
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 460800;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART1_Init 2 */
    
    /* USER CODE END USART1_Init 2 */
    
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{
    
    /* USER CODE BEGIN USART2_Init 0 */
    
    /* USER CODE END USART2_Init 0 */
    
    /* USER CODE BEGIN USART2_Init 1 */
    
    /* USER CODE END USART2_Init 1 */
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 460800;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART2_Init 2 */
    
    /* USER CODE END USART2_Init 2 */
    
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{
    
    /* USER CODE BEGIN USART3_Init 0 */
    
    /* USER CODE END USART3_Init 0 */
    
    /* USER CODE BEGIN USART3_Init 1 */
    
    /* USER CODE END USART3_Init 1 */
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 460800;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART3_Init 2 */
    
    /* USER CODE END USART3_Init 2 */
    
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{
    
    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();
    
    /* DMA interrupt init */
    /* DMA1_Channel2_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
    /* DMA1_Channel3_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
    /* DMA1_Channel4_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
    /* DMA1_Channel5_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
    
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    
    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    
    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(parachute_GPIO_Port, parachute_Pin, GPIO_PIN_RESET);
    
    /*Configure GPIO pin : PC13 */
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    
    /*Configure GPIO pin : PA4 */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /*Configure GPIO pin : parachute_Pin */
    GPIO_InitStruct.Pin = parachute_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(parachute_GPIO_Port, &GPIO_InitStruct);
    
}

/* USER CODE BEGIN 4 */
//обработчик прерывания
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim==&htim2)
    {
        Timer1++;
    }
}

void HAL_UART_IDLE_Callback(UART_HandleTypeDef *huart)
{
    if(huart == &huart1)
    {
        __HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);
        rx_buff_len = BUFSIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);
        //HAL_UART_DMAStop(&huart1);
        
        
        
        uint8_t res = HAL_UART_Transmit_DMA(&huart1, (uint8_t*)rx_buff, rx_buff_len);
        if(res == HAL_ERROR) HAL_UART_Transmit(&huart1, (uint8_t*)"HAL_ERROR - rx_buff == NULL or rx_buff_len == 0\n", 48, 1000);
        else if(res == HAL_BUSY) HAL_UART_Transmit(&huart1, (uint8_t*)"HAL_BUSY\n", 9, 1000);
        HAL_UART_AbortReceive(&huart1);
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);
        __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
        HAL_UART_Receive_DMA(&huart1, (uint8_t*)rx_buff, BUFSIZE);
    }
}
//


void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart1)
    {
        __HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);
        HAL_UART_DMAStop(&huart1);
        //		HAL_GPIO_TogglePin(pc13_GPIO_Port, pc13_Pin);
        uint32_t er = HAL_UART_GetError(&huart1);
        
        switch(er)
        {
            case HAL_UART_ERROR_PE:
                HAL_UART_Transmit(&huart1, (uint8_t*)"ERR_Callbck - Parity error\n", 27, 1000);
                __HAL_UART_CLEAR_PEFLAG(&huart1);
                huart->ErrorCode = HAL_UART_ERROR_NONE;
                break;
                
            case HAL_UART_ERROR_NE:
                HAL_UART_Transmit(&huart1, (uint8_t*)"ERR_Callbck - Noise error\n", 26, 1000);
                __HAL_UART_CLEAR_NEFLAG(&huart1);
                huart->ErrorCode = HAL_UART_ERROR_NONE;
                break;
                
            case HAL_UART_ERROR_FE:
                HAL_UART_Transmit(&huart1, (uint8_t*)"ERR_Callbck - Frame error\n", 26, 1000);
                __HAL_UART_CLEAR_FEFLAG(&huart1);
                huart->ErrorCode = HAL_UART_ERROR_NONE;
                break;
                
            case HAL_UART_ERROR_ORE:
                HAL_UART_Transmit(&huart1, (uint8_t*)"ERR_Callbck - Overrun error\n", 28, 1000);
                __HAL_UART_CLEAR_OREFLAG(huart);
                huart->ErrorCode = HAL_UART_ERROR_NONE;
                break;
                
            case HAL_UART_ERROR_DMA:
                HAL_UART_Transmit(&huart1, (uint8_t*)"ERR_Callbck - DMA transfer error\n", 33, 1000);
                huart->ErrorCode = HAL_UART_ERROR_NONE;
                break;
                
            default:
                break;
        }
    }
    
}



/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
