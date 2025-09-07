/* USER CODE BEGIN Header */
//##############################################################################
// ���j�o�[�T���L�b�g(Universal Kit) STM-Version
// �T���v���v���O���� ver.1.0 (2023.7.24-)
//
// sample12 Trace_Speed(���[�^�o�͂Q�F��`����)���g���[�T�p
//##############################################################################
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"	// C�̕W�����C�u����
#include "LCD_c.h"	// LCD�p
#include "Beep.h"	// Beep�p

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//******************************************************************************
// �}�N����`
//******************************************************************************
// �X�C�b�`�֘A
#define LED_ON		GPIO_PIN_SET		// LED(LD3)�͐��_��ActiveHigh�Ȃ̂�1��ON
#define LED_OFF		GPIO_PIN_RESET		// LED(LD3)�͐��_��ActiveHigh�Ȃ̂�0��OFF
#define SW_ON		GPIO_PIN_RESET		// Switch�͕��_��ActiveLow�Ȃ̂�0��ON
#define SW_OFF		GPIO_PIN_SET		// Switch�͕��_��ActiveLow�Ȃ̂�1��OFF
#define SW_WAIT     300					// �`���^�����O�h�~�p�̑҂�����(ms)
// ���[�h�֘A
#define MODE_MAX	9					// ���샂�[�h��
#define DISP		0					// ���[�h�\��
#define EXEC		1					// ���[�h���s
// �Z���T�֘A
#define SEN_WAIT    20         			// �Z���T�������Ԓ���
// ���[�^�֘A
#define MOT_ON      1         			// ���[�^�d�� ON
#define MOT_OFF     0         			// ���[�^�d�� OFF
#define MOT_L_FWD   1         			// �����[�^�O�i
#define MOT_L_BACK  0         			// �����[�^��i
#define MOT_R_FWD   0         			// �E���[�^�O�i
#define MOT_R_BACK  1         			// �E���[�^��i
#define MOT_SPD_INIT    100   			// ���[�^�N�����x

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//******************************************************************************
// �O���[�o���ϐ�
//******************************************************************************
int tick_count = 0;						// ���荞�݃J�E���g�p�ϐ�
int time_count = 0;						// �o�ߎ��ԃJ�E���g�p�ϐ�
float batt;								// �d���d��
int mode = 0;							// ���݃��[�h�i�[�p
int sen_val[8];							// �Z���T�l	7 6 5 4 3 2 1 ���E����i���o�����O
int sen_ref[8];							// ���C���g���[�X�p �Z���T臒l
uint16_t adc1_val[4];                   // ADC1��ϊ������l���i�[����z��
uint16_t adc2_val[4];                   // ADC2��ϊ������l���i�[����z��
int mot_spd_r = 0;            			// �E���[�^���x
int mot_spd_l = 0;            			// �����[�^���x
int target_spd_r = 0;          			// �E���[�^�ڕW���x
int target_spd_l = 0;          			// �����[�^�ڕW���x
int target_spd = 0;
int pre_sen_val[8];

int MOT_ACC = 0;						// ���[�^�����x

int sen_val_r = 0;            // �E�Z���T�l
int sen_val_f = 0;            // �O�Z���T�l
int sen_val_l = 0;            // ���Z���T�l
int sen_val_ml = 0;            // �E�Z���T�l
int sen_val_mr = 0;            // ���Z���T�l
int sen_val_gr = 0;
int line_val = 0;					//���C����Ԋi�[�ϐ�
int pre_line_val;

int R_middle_SEN = 0;
int L_middle_SEN = 0;
int R_SEN = 0;
int F_SEN = 0;
int L_SEN = 0;

//�Е���line_val����������
int hosei_r_flag = 0;
int hosei_l_flag = 0;


//�L�����u���[�V�����p�̕ϐ�(�ő�E�ŏ�)
int sen_val_pre_max[8];
int sen_val_pre_min[8];

/*int sen_val_pre_max[1] = 999;
int sen_val_pre_max[2] = 999;
int sen_val_pre_max[3] = 999;
int sen_val_pre_max[4] = 999;
int sen_val_pre_max[5] = 999;
int sen_val_pre_max[6] = 999;
int sen_val_pre_max[7] = 999;

int sen_val_pre_min[1] = 0;
int sen_val_pre_min[2] = 0;
int sen_val_pre_min[3] = 0;
int sen_val_pre_min[4] = 0;
int sen_val_pre_min[5] = 0;
int sen_val_pre_min[6] = 0;
int sen_val_pre_min[7] = 0;*/

//���΍��E�E�΍��ۑ��p�ϐ�
int sen_diff_r = 0;
int sen_diff_l = 0;
int sen_diff_ml = 0;
int sen_diff_mr = 0;
int pre_sen_diff_r = 0;     //�ЂƂO�̕΍���ۑ�
int pre_sen_diff_l = 0;     //�ЂƂO�̕΍���ۑ�
int pre_sen_diff_ml = 0;
int pre_sen_diff_mr = 0;

int line_counter = 0;
int line_flag_r= 0;
int line_flag_l= 0;

//�L�����u���[�V�����v�Z�p�ϐ�
float calibracion_r = 0;        //�E�Z���T
float calibracion_f = 0;        //�����Z���T
float calibracion_l = 0;        //���Z���T


//���C���g���[�X�p����Q�C��
float trace_gain_f    = 0;          // ���C���g���[�X�p�@����Q�C���F����
float trace_gain_fr   = 0;          // ���C���g���[�X�p�@����Q�C���F�����{�E
float trace_gain_fl   = 0;          // ���C���g���[�X�p�@����Q�C���F�����{��
float trace_gain_r    = 0;          // ���C���g���[�X�p�@����Q�C���F�E
float trace_gain_l    = 0;          // ���C���g���[�X�p�@����Q�C���F��
float trace_gain_r_ext= 0;          // ���C���g���[�X�p�@����Q�C���F�E�Z���T�O
float trace_gain_l_ext= 0;          // ���C���g���[�X�p�@����Q�C���F���Z���T�O

float control_r = 0;
float control_l = 0;


int r_out_flg;
int l_out_flg;
int ext_flag_r;			//	�Z���T�O�Ή�
int ext_flag_l;

int goal;                     // �S�[���}�[�J�p�@�ϐ��J�E���g
int c_counter;                // �^�񒆂̃J�E���g
int s_handan;               //�N���X�������łȂ����f�p�ϐ�
int cross_handan =0;          //�S�[�����f���̃N���X���f���邽�߂̕ϐ�
int goal_extend = 0;            //�S�[�������Ɣ��f�����ۂ̂܂��������邽�߂̔��f�p�ϐ�
int goal_chase = 0;             //�S�[������ۂ̎��Ԑ�����悤�ϐ��D
int cross_flag = 0;

int cross_count_R;				//�S�[���J�E���g
int goal_count = 0;					//�X�^�[�g����̌o�ߎ���


//���݂̃Z���T�l�ۑ��p�ϐ�(�L���������[�V�����̉e����P���䂪�o���Ȃ�����)
int sen_hozon_f = 0;

int sen_mokuhyou = 999;
int hensa_hozon = 0;
float hensa_Kp = 0.8;

float Kp = 0;           //���Q�C���錾
float Kd = 0;
float d;

int id_flag = 0;        //D����̎��Ԃ𐔂��邽�߂̃t���O
int id_cnt = 0;         //D����̎��Ԃ𐔂��邽�߂̊i�[�ϐ�
int pre_id_cnt;

int i = 0;


//���䎮�̕ϐ�
float seigyo_p = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void finish( void );
void select_mode( int mode_com );
void mot_l_drive( void );
void mot_r_drive( void );
void sen_ref_all( void );
void calibration( void );
void set_param( void );
void second_param( void );
void third_param( void );
void fourth_param( void );
void fifth_param( void );
void countdown( void );
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_I2C1_Init();
  MX_TIM6_Init();
  MX_TIM16_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  LCD_init();								// LCD ������
  HAL_TIM_Base_Start_IT(&htim6);			// �^�C�}�X�^�[�g�֐�
  HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED); // ADC1 �L�����u���[�V����
  HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED); // ADC2 �L�����u���[�V����
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1_val, 4);     // DMA�]���J�n
  HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc2_val, 4);     // DMA�]���J�n

  // �N��������
  Beep(TONE_RE,6,100);   					// �N����:����5�̃�, 100ms
  Beep(TONE_SO,6,100);						// �N����:����5�̃\, 100ms
  LCD_print(1, 0, "Sample12: Speed ");		// LCD1�s�ڃf�[�^�Z�b�g
  LCD_print(2, 0, "       .  [ V ] ");		// LCD2�s�ڃf�[�^�Z�b�g

  // �d���d���擾
  batt = adc1_val[3] * 3.3 / 1023.0 * 10.0; // �擾�l��d���ɕϊ��F������1/10��߂����߂�10�{
  LCD_dec_out(2, 5, (int)batt, 2);			// �d��������2���\��
  LCD_dec_out(2, 8, (int)((batt-(int)batt)*10), 1);	// �d����������1���\��
  HAL_Delay( 2000 );

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if( HAL_GPIO_ReadPin(SW_UP_GPIO_Port, SW_UP_Pin) == SW_ON ){	// SW_UP�������ꂽ��
      HAL_Delay(SW_WAIT);											// �`���^�����O�h�~
      mode++;								// ���[�h��1�グ��
	}
    if( HAL_GPIO_ReadPin(SW_DOWN_GPIO_Port, SW_DOWN_Pin) == SW_ON ){// SW_DOWN�������ꂽ��
      HAL_Delay(SW_WAIT);											// �`���^�����O�h�~
      mode--;								// ���[�h��1������
	}
    if( HAL_GPIO_ReadPin(SW_MID_GPIO_Port, SW_MID_Pin) == SW_ON ){	// SW_MID�������ꂽ��
      HAL_Delay(SW_WAIT);											// �`���^�����O�h�~
      select_mode( EXEC );					// ���[�h���s
      mode = 0;								// ���s��͏�����ʂɖ߂�
    }
    select_mode( DISP );					// �I�𒆂̃��[�h��\��

    // ������h�~
  }
  /* USER CODE END 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//==============================================================================
// �e���[�h
//==============================================================================
void select_mode( int mode_com ){
  //���[�h�ԍ��F����^��������
  if( mode >= MODE_MAX )  mode = 0;		// ���[�h���ő�l�𒴂��Ă���ꍇ��0�ɖ߂�
  if( mode < 0 )  mode = MODE_MAX - 1;	// ���[�h�����̏ꍇ�̓��[�h���ő�l�ɐݒ�

  if( mode == 0 ){
  	//------------------------------------------------
  	// Mode0 : �Z���T�`�F�b�N
  	//------------------------------------------------
  	if( mode_com == DISP ){  			// DISP�w���̏ꍇ�F���[�h�^�C�g���\��
  	  LCD_print( 1, 0, "0: Sensor Check " );
  	  LCD_print( 2, 0, "                " );
  	}else if( mode_com == EXEC ){		// EXEC�w���̏ꍇ�F���[�h���s
  		LCD_clear(1);
  		do{
  			LCD_dec_out( 2, 12, sen_val[1], 3 );	// �E�[�Z���T�l	�~�@�~�@�~�@�~�@�~�@�~�@�Z
  			LCD_dec_out( 1, 10, sen_val[2], 3 );	// �E2�ڃZ���T�l	�~�@�~�@�~�@�~�@�~�@�Z�@�~
  			LCD_dec_out( 2,  8, sen_val[3], 3 );	// �����E�Z���T�l	�~�@�~�@�~�@�~�@�Z�@�~�@�~
  			LCD_dec_out( 1,  6, sen_val[4], 3 );	// �����Z���T�l	�~�@�~�@�~�@�Z�@�~�@�~�@�~
  			LCD_dec_out( 2,  4, sen_val[5], 3 );	// �������Z���T�l	�~�@�~�@�Z�@�~�@�~�@�~�@�~
  			LCD_dec_out( 1,  2, sen_val[6], 3 );	// ��2�ڃZ���T�l	�~�@�Z�@�~�@�~�@�~�@�~�@�~
  			LCD_dec_out( 2,  0, sen_val[7], 3 );	// ���[�Z���T�l	�Z�@�~�@�~�@�~�@�~�@�~�@�~
  		}while( HAL_GPIO_ReadPin(SW_MID_GPIO_Port, SW_MID_Pin) == SW_OFF );
  		// SW_MID�������ꂽ��
  		HAL_Delay(SW_WAIT);
  		LCD_clear(2);
  	}
    }
    else if( mode == 1 ){
  	//------------------------------------------------
  	// Mode1 : ���[�^�`�F�b�N
  	//------------------------------------------------
  	if( mode_com == DISP ){  			// DISP�w���̏ꍇ�F���[�h�^�C�g���\��
  	  LCD_print( 1, 0, "1: Motor Check  " );
  	  LCD_print( 2, 0, "                " );
  	}else if( mode_com == EXEC ){		// EXEC�w���̏ꍇ�F���[�h���s

  		LCD_print( 2, 0, " SPD =          " );
  		HAL_GPIO_WritePin(MOT_EN_GPIO_Port, MOT_EN_Pin, MOT_ON);			// ���[�^Enable
  		HAL_GPIO_WritePin(MOT_R_DIR_GPIO_Port, MOT_R_DIR_Pin, MOT_R_FWD);	// �E���[�^�����w�� : �O�i
  	    HAL_GPIO_WritePin(MOT_L_DIR_GPIO_Port, MOT_L_DIR_Pin, MOT_L_FWD);	// �����[�^�����w�� : �O�i

  	    while( 1 ){
  	    	LCD_dec_out( 2,  2, target_spd_r, 4 );							// �w�����x�\���@����\�ŉE��\��
  	    	if( HAL_GPIO_ReadPin(SW_UP_GPIO_Port, SW_UP_Pin) == SW_ON ){	// SW_UP�������ꂽ��
  	        	HAL_Delay(SW_WAIT);											// �`���^�����O�h�~
  	        	target_spd_r += 100;										// �E����
  	        	target_spd_l += 100;										// ������
  	    	}
  	        if( HAL_GPIO_ReadPin(SW_DOWN_GPIO_Port, SW_DOWN_Pin) == SW_ON ){// SW_DOWN�������ꂽ��
  	        	HAL_Delay(SW_WAIT);											// �`���^�����O�h�~
  	        	target_spd_r -= 100;										// �E����
  	        	target_spd_l -= 100;										// ������
  	    	}
  	        if( HAL_GPIO_ReadPin(SW_MID_GPIO_Port, SW_MID_Pin) == SW_ON ){	// SW_MID�������ꂽ��
  	        	HAL_Delay(SW_WAIT);											// �`���^�����O�h�~
  	        	target_spd_r = 0;											// �E��~
  	        	target_spd_l = 0;											// ����~
  	        	while( mot_spd_l != 0 || mot_spd_r != 0 );    				// ���S��~�҂�
  	    	    HAL_GPIO_WritePin(MOT_EN_GPIO_Port, MOT_EN_Pin, MOT_OFF);	// ���[�^Disable
  	    	    break;
  	        }
  	    }
  	}
    }
    else if( mode == 2 ){
  	//------------------------------------------------
  	// Mode2 :
  	//------------------------------------------------
  	if( mode_com == DISP ){  			// DISP�w���̏ꍇ�F���[�h�^�C�g���\��
  	  LCD_print( 1, 0, "2: sen_ref        " );
  	  LCD_print( 2, 0, "                " );
  	}else if( mode_com == EXEC ){		// EXEC�w���̏ꍇ�F���[�h���s
  		LCD_clear(1);
  		sen_ref_all();
  		do{
  			if( sen_val[1] > sen_ref[1] )	LCD_print( 2, 12, "�Z" );		// �E�[�Z���T�l	�~�@�~�@�~�@�~�@�~�@�~�@�Z
  			else 								LCD_print( 2, 12, "�~" );
  			if( sen_val[2] > sen_ref[2] )	LCD_print( 1, 10, "�Z" );		// �E2�ڃZ���T�l	�~�@�~�@�~�@�~�@�~�@�Z�@�~
  			else 								LCD_print( 1, 10, "�~" );
  			if( sen_val[3] > sen_ref[3] )	LCD_print( 2, 8, "�Z" );		// �����E�Z���T�l	�~�@�~�@�~�@�~�@�Z�@�~�@�~
  			else 								LCD_print( 2, 8, "�~" );
  			if( sen_val[4] > sen_ref[4] )	LCD_print( 1, 6, "�Z" );		// �����Z���T�l	�~�@�~�@�~�@�Z�@�~�@�~�@�~
  			else 								LCD_print( 1, 6, "�~" );
  			if( sen_val[5] > sen_ref[5] )	LCD_print( 2, 4, "�Z" );		// �������Z���T�l	�~�@�~�@�Z�@�~�@�~�@�~�@�~
  			else 								LCD_print( 2, 4, "�~" );
  			if( sen_val[6] > sen_ref[6] )	LCD_print( 1, 2, "�Z" );		// ��2�ڃZ���T�l	�~�@�Z�@�~�@�~�@�~�@�~�@�~
  			else 								LCD_print( 1, 2, "�~" );
  			if( sen_val[7] > sen_ref[7] )	LCD_print( 2, 0, "�Z" );		// ��2�ڃZ���T�l	�~�@�Z�@�~�@�~�@�~�@�~�@�~
  			else 								LCD_print( 2, 0, "�~" );

  		}while( HAL_GPIO_ReadPin(SW_MID_GPIO_Port, SW_MID_Pin) == SW_OFF );
  		 // SW_MID�������ꂽ��
  		 HAL_Delay(SW_WAIT);
  		 LCD_clear(2);
  	  //LCD_print( 2, 0, "    OK!!!!!!    " );
  	  //HAL_Delay( 1000 );
  	}
    }
    else if( mode == 3 ){
  	//------------------------------------------------
  	// Mode 3:���s���[�h
  	//------------------------------------------------
  	  if( mode_com == DISP ){  			// DISP�w���̏ꍇ�F���[�h�^�C�g���\��
  	  	  LCD_print( 1, 0, "3: tida    Trace" );
  	  	  LCD_print( 2, 0, "                " );
  	  	}else if( mode_com == EXEC ){		// EXEC�w���̏ꍇ�F���[�h���s
  	  		LCD_clear(1);
  	  		set_param();
  	  		sen_ref_all();

  	  		while(1){
  				if(HAL_GPIO_ReadPin(SW_UP_GPIO_Port, SW_UP_Pin) == SW_ON ){// �ESW���͔��f
  					HAL_Delay( SW_WAIT );        // �`���^�����O�h�~
  					break;
  				}
  				if( HAL_GPIO_ReadPin(SW_DOWN_GPIO_Port, SW_DOWN_Pin) == SW_ON ){// ��SW���͔��f
  					HAL_Delay( SW_WAIT );        // �`���^�����O�h�~
  					calibration();
  					//LCD_print( 1, 0, "3: calibration" );
  					HAL_Delay( 1000 );
  					break;
  				}
  			}

  	  		goal_count = 0;
  			s_handan = 0;
  			cross_handan = 0;
  			countdown();
  			cross_count_R = 0;

  	  	    HAL_GPIO_WritePin(MOT_EN_GPIO_Port, MOT_EN_Pin, MOT_ON);			// ���[�^Enable
  		    HAL_GPIO_WritePin(MOT_R_DIR_GPIO_Port, MOT_R_DIR_Pin, MOT_R_FWD);	// �E���[�^�����w�� : �O�i
  		    HAL_GPIO_WritePin(MOT_L_DIR_GPIO_Port, MOT_L_DIR_Pin, MOT_L_FWD);	// �����[�^�����w�� : �O�i
  		    while(1){
  			   //do{

					   // �Z���T�l���烉�C����Ԃ�ۑ�
					   line_val = 0;                             // ���C����ԃ��Z�b�g

					   sen_val_ml =  sen_val[6];       // �E�Z���T��A/D���l��0-999�ɉ��H
					   sen_val_mr =  sen_val[2];       // �O�Z���T��A/D���l��0-999�ɉ��H
					   sen_val_r =  sen_val[3];       // �E�Z���T��A/D���l��0-999�ɉ��H
					   sen_val_f =  sen_val[4];       // �O�Z���T��A/D���l��0-999�ɉ��H
					   sen_val_l =  sen_val[5];       // ���Z���T��A/D���l��0-999�ɉ��H
					   sen_val_gr = sen_val[1];       // ���Z���T��A/D���l��0-999�ɉ��H

					   R_SEN = (float)(sen_val_r - sen_val_pre_min[3])*1000/(sen_val_pre_max[3] - sen_val_pre_min[3]);
					   F_SEN = (float)(sen_val_f - sen_val_pre_min[4])*1000/(sen_val_pre_max[4] - sen_val_pre_min[4]);
					   L_SEN = (float)(sen_val_l - sen_val_pre_min[5])*1000/(sen_val_pre_max[5] - sen_val_pre_min[5]);
					   L_middle_SEN = (float)(sen_val_ml - sen_val_pre_min[6])*1000/(sen_val_pre_max[6] - sen_val_pre_min[6]);
					   R_middle_SEN = (float)(sen_val_mr - sen_val_pre_min[2])*1000/(sen_val_pre_max[2] - sen_val_pre_min[2]);

					   /*if( R_SEN == 1 )     line_val += 1;  // �E�Z���T
					   if( F_SEN == 1 )     line_val += 2;  // �����Z���T
					   if( L_SEN == 1 )     line_val += 4;  // ���Z���T*/

					   //PD_control(L_SEN ,F_SEN ,R_SEN );

					   if( R_SEN > sen_ref[3] )     line_val += 2;  // �E�Z���T
					   if( F_SEN > sen_ref[4] )     line_val += 4;  // �����Z���T
					   if( L_SEN > sen_ref[5] )     line_val += 10;  // ���Z���T
					   if( R_middle_SEN > sen_ref[2])	line_val += 1;
					   if( L_middle_SEN > sen_ref[6])	line_val += 11;


					   if( ext_flag_r == 1 ){
						   if(((( line_val == 1 || line_val == 3 )|| line_val == 6) || line_val == 7) || line_val == 4 ) {ext_flag_r = 0;}
					   }
					   else if( ext_flag_l == 1 ){
						   if(((( line_val == 11 || line_val == 21 ) || line_val == 25) || line_val == 14) || line_val == 4){ ext_flag_l = 0;}
					   }

					   if( ext_flag_r == 1 ){
						   pre_line_val = 1;
						   line_val = 0;
					   }
					   else if( ext_flag_l == 1 ){
						   pre_line_val = 11;
						   line_val = 0;
					   }
					   // �Z���T�O�Ή�
					   if( ( pre_line_val == 1 || pre_line_val == 30 ) && line_val == 0 )
						   line_val = 30;   // 1�O�̏�Ԃ��E�̂݁C�܂��͉E�Z���T�O�Ή��ŁC�����S�Z���T�������̏ꍇ�F�E�Z���T�O�Ή�
					   if( ( pre_line_val == 11 || pre_line_val == 31 ) && line_val == 0 )
						   line_val = 31;   // 1�O�̏�Ԃ����̂݁C�܂��͍��Z���T�O�Ή��ŁC�����S�Z���T�������̏ꍇ�F���Z���T�O�Ή�

					   LCD_dec_out( 2,  1, target_spd_l, 4 );	// ���[�Z���T�l	�Z�@�~�@�~�@�~�@�~�@�~�@�~
					   LCD_dec_out( 2,  12, target_spd_r, 4 );	// ���[�Z���T�l	�Z�@�~�@�~�@�~�@�~�@�~�@�~
					   LCD_dec_out( 1,  7, cross_count_R, 2 );	// ���[�Z���T�l	�Z�@�~�@�~�@�~�@�~�@�~�@�~

					   // ���C����Ԃ��烂�[�^�ڕW���x�ݒ�
					   switch( line_val ){

						   case 1: // line_val = 1 : XXO : �E�Z���T�̂ݔ���
								   sen_diff_mr = ( sen_val_r - sen_val_mr )-845;
								   d = (sen_diff_mr - pre_sen_diff_mr)/ pre_id_cnt;
								   control_r = (sen_diff_mr * Kp) + (d * Kd);
								   cross_count_R = 0;
								   target_spd_r = (target_spd + control_r)-650;
								   target_spd_l = target_spd;
								   break;

						   case 2: // line_val = 2 : XOO : �����E�{�E�Z���T����
								   sen_diff_r = ( sen_val_f - sen_val_r )-845;
								   d = (sen_diff_r - pre_sen_diff_r)/ pre_id_cnt;
								   control_r = (sen_diff_r * Kp) + (d * Kd);
								   cross_count_R = 0;
								   target_spd_r = (target_spd + control_r)-300;
								   target_spd_l = target_spd;
								   line_flag_r = 1;
								   break;


						   case 3: // line_val = 3 : XOO : �����E�{�E�Z���T����
								   sen_diff_mr = ( sen_val_r - sen_val_mr )-845;
								   d = (sen_diff_mr - pre_sen_diff_mr)/ pre_id_cnt;
								   control_r = (sen_diff_mr * Kp) + (d * Kd);
								   cross_count_R = 0;
								   target_spd_r = (target_spd + control_r)-300;
								   target_spd_l = target_spd;
								   break;

						   case 4: // line_val = 4 : OXX : �����Z���T�̂ݔ���
								   target_spd_r = target_spd;
								   target_spd_l = target_spd;
								   break;

						   case 5: // line_val = 3 : XOO : �����E�{�E�Z���T����
								   sen_diff_mr = ( sen_val_r - sen_val_mr )-845;
								   d = (sen_diff_mr - pre_sen_diff_mr)/ pre_id_cnt;
								   control_r = (sen_diff_mr * Kp) + (d * Kd);
								   cross_count_R = 0;
								   target_spd_r = target_spd + control_r;
								   target_spd_l = target_spd;
								   break;

						   case 6: // line_val = 6 : OOX :����+�����E�Z���T����
								   sen_diff_r = ( sen_val_f - sen_val_r )-845;
								   d = (sen_diff_r - pre_sen_diff_r)/ pre_id_cnt;
								   control_r = (sen_diff_r * Kp) + (d * Kd);
								   cross_count_R = 0;
								   target_spd_r = target_spd + control_r;
								   target_spd_l = target_spd;
								   break;

						   /*case 7: // line_val = 6 : OOX :����+�����E�Z���T����
								   sen_diff_l = ( sen_val_l - sen_val_f )+845;
								   d = (sen_diff_l - pre_sen_diff_l)/ pre_id_cnt;
								   control_l = (sen_diff_l * Kp) + (d * Kd);
								   cross_count_R = 0;
								   line_flag_l = 1;
								   target_spd_r = target_spd;
								   target_spd_l = target_spd - control_l;
								   break;*/

						   case 14: // line_val = 11 : OOX : �����{�������Z���T����
								   sen_diff_l = ( sen_val_l - sen_val_f )+845;
								   d = (sen_diff_l - pre_sen_diff_l)/ pre_id_cnt;
								   control_l = (sen_diff_l * Kp) + (d * Kd);
								   cross_count_R = 0;
								   target_spd_r = target_spd;
								   target_spd_l = target_spd - control_l;
								   break;

						   case 15:// line_val = 12: OOX :���Z���T����
								   sen_diff_ml = ( sen_val_ml - sen_val_l )+845;
								   d = (sen_diff_ml - pre_sen_diff_ml)/ pre_id_cnt;
								   control_l = (sen_diff_ml * Kp) + (d * Kd);
								   cross_count_R = 0;
								   target_spd_r = target_spd;
								   target_spd_l = target_spd - control_l;
								   break;


						   case 16: // line_val = 16 : OXX : �����Z���T�̂ݔ���
								   target_spd_r = target_spd;
								   target_spd_l = target_spd;
								   break;

						   case 28: // line_val = 16 : OXX : �����Z���T�̂ݔ���
								   target_spd_r = target_spd;
								   target_spd_l = target_spd;
								   break;

						   case 12: // line_val = 16 : OXX : �����Z���T�̂ݔ���
								   target_spd_r = target_spd;
								   target_spd_l = target_spd;
								   break;


						   case 17:// line_val = 19: OOX : �������{���Z���T����
								   sen_diff_r = ( sen_val_f - sen_val_r )-845;
								   d = (sen_diff_r - pre_sen_diff_r)/ pre_id_cnt;
								   control_r = (sen_diff_r * Kp) + (d * Kd);
								   cross_count_R = 0;
								   target_spd_r = target_spd + control_r;
								   target_spd_l = target_spd;
								   break;


						   case 27:// line_val = 19: OOX : �������{���Z���T����
								   sen_diff_l = ( sen_val_l - sen_val_f )+845;
								   d = (sen_diff_l - pre_sen_diff_l)/ pre_id_cnt;
								   control_l = (sen_diff_l * Kp) + (d * Kd);
								   cross_count_R = 0;
								   target_spd_r = target_spd;
								   target_spd_l = target_spd - control_l;
								   break;


						   case 10:// line_val = 19: OOX : �������{���Z���T����
								   sen_diff_l = ( sen_val_l - sen_val_f )+845;
								   d = (sen_diff_l - pre_sen_diff_l)/ pre_id_cnt;
								   control_l = (sen_diff_ml * Kp) + (d * Kd);
								   cross_count_R = 0;
								   target_spd_r = target_spd;
								   target_spd_l = (target_spd - control_l)-300;
								   break;

						   case 11:// line_val = 12: OOX :���Z���T����
								   sen_diff_ml = ( sen_val_ml - sen_val_l )+845;
								   d = (sen_diff_ml - pre_sen_diff_ml)/ pre_id_cnt;
								   control_l = (sen_diff_ml * Kp) + (d * Kd);
								   cross_count_R = 0;
								   target_spd_r = target_spd;
								   target_spd_l = (target_spd - control_l)-650;
								   break;


						   case 13: // line_val = 3 : XOO : �����E�{�E�Z���T����
								   sen_diff_mr = ( sen_val_r - sen_val_mr )-845;
								   d = (sen_diff_mr - pre_sen_diff_mr)/ pre_id_cnt;
								   control_r = (sen_diff_mr * Kp) + (d * Kd);
								   cross_count_R = 0;
								   target_spd_r = target_spd + control_r;
								   target_spd_l = target_spd;
								   break;


						   case 21:// line_val = 12: OOX :���Z���T����
								   sen_diff_ml = ( sen_val_ml - sen_val_l )+845;
								   d = (sen_diff_ml - pre_sen_diff_ml)/ pre_id_cnt;
								   control_l = (sen_diff_ml * Kp) + (d * Kd);
								   cross_count_R = 0;
								   target_spd_r = target_spd;
								   target_spd_l = (target_spd - control_l)-300;
								   break;

						   case 30: // line_val = 8 : XXX : �E�Z���T�O�Ή�
								   //target_spd_r = target_spd - ((F_SENP-sen_val_f)*1.5);
								   ext_flag_r = 1;
								   cross_count_R = 0;
								   //trace_gain_r_ext= 0.28;
								   target_spd_r = 300;//target_spd_r * trace_gain_r_ext;
								   target_spd_l = target_spd;//1.1
								   break;

						   case 31: // line_val = 9 : XXX : ���Z���T�O�Ή�
								   ext_flag_l = 1;
								   cross_count_R = 0;
								   //trace_gain_l_ext = 0.28;
								   target_spd_r = target_spd;//1.1
								   target_spd_l = 300;//target_spd * trace_gain_l_ext;
								   break;

						   default:// line_val = 0,5,7 : XXX, OXO, OOO : ��~
								   target_spd_r = 0;
								   target_spd_l = 0;
								   //cross_flg = 1;
								   break;
					   }
  						   LCD_dec_out( 2,  1, target_spd_l, 4 );	// ���[�Z���T�l	�Z�@�~�@�~�@�~�@�~�@�~�@�~
  						   LCD_dec_out( 2,  12, target_spd_r, 4 );	// ���[�Z���T�l	�Z�@�~�@�~�@�~�@�~�@�~�@�~
  						   LCD_dec_out( 1,  7, cross_count_R, 5 );	// ���[�Z���T�l	�Z�@�~�@�~�@�~�@�~�@�~�@�~

  							   // ���C����ԕۑ�
  							   pre_line_val = line_val;
  							   pre_sen_diff_r = sen_diff_r;
  							   pre_sen_diff_l = sen_diff_l;
  							   pre_sen_diff_ml = sen_diff_ml;
  							   pre_sen_diff_mr = sen_diff_mr;

  							   if(sen_val[3] > 400 && sen_val[4] > 400 &&sen_val[5] > 400) {
  								   cross_handan = 1;
  								   goal_count = 0;
  							   }

  							   if(s_handan == 0 && sen_val[1] > sen_ref[1] ){
  								   cross_handan = 1;
  								   s_handan = 1;
  							   }

  							   if( goal_count > 2000 ){
  								   cross_handan = 0;
  								   //cross_count_R = 0;
  							   }
  							   if(sen_val[1] > sen_ref[1]){
  								   if( 1 < cross_count_R && cross_count_R < 100 ){
  									   //cross_count_R = 0;
  									   if( cross_handan == 0 && s_handan == 1){
  										   target_spd_r = 1400;
  										   target_spd_l = 1400;
  										   HAL_Delay(300);
  										   target_spd_r = 1000;
  										   target_spd_l = 1000;
  										   HAL_Delay(300);
  										   target_spd_r = 500;
  										   target_spd_l = 500;
  										   HAL_Delay(100);
  										   break;
  									   }
  								   }else{
  									   //cross_count_R = 0;
  								   }
  							   }
  						   if(HAL_GPIO_ReadPin(SW_MID_GPIO_Port, SW_MID_Pin) == SW_ON){
  							   HAL_Delay( SW_WAIT );
  							   break;
  						   }


  					   }//while(  HAL_GPIO_ReadPin(SW_MID_GPIO_Port, SW_MID_Pin) == SW_ON );         // ����SW���͔��f
  				   //HAL_Delay( SW_WAIT );                          // �`���^�����O�h�~
  				   target_spd_r = 0;
  				   target_spd_l = 0;
  				   while( mot_spd_l != 0 || mot_spd_r != 0 );    // ���S��~�҂�
  				   HAL_GPIO_WritePin(MOT_EN_GPIO_Port, MOT_EN_Pin, MOT_OFF);                  // ���[�^Enable OFF

  	  	}
    }
    else if( mode == 4 ){
    	//------------------------------------------------
    	// Mode 4:���s���[�h
    	//------------------------------------------------
    	  if( mode_com == DISP ){  			// DISP�w���̏ꍇ�F���[�h�^�C�g���\��
    	  	  LCD_print( 1, 0, "4: Second Trace" );
    	  	  LCD_print( 2, 0, "                " );
    	  	}else if( mode_com == EXEC ){		// EXEC�w���̏ꍇ�F���[�h���s
    	  		LCD_clear(1);
    	  		second_param();
    	  		sen_ref_all();

    	  		while(1){
    				if(HAL_GPIO_ReadPin(SW_UP_GPIO_Port, SW_UP_Pin) == SW_ON ){// �ESW���͔��f
    					HAL_Delay( SW_WAIT );        // �`���^�����O�h�~
    					break;
    				}
    				if( HAL_GPIO_ReadPin(SW_DOWN_GPIO_Port, SW_DOWN_Pin) == SW_ON ){// ��SW���͔��f
    					HAL_Delay( SW_WAIT );        // �`���^�����O�h�~
    					calibration();
    					//LCD_print( 1, 0, "3: calibration" );
    					HAL_Delay( 1000 );
    					break;
    				}
    			}

    	  		goal_count = 0;
    			s_handan = 0;
    			cross_handan = 0;
    			countdown();
    			cross_count_R = 0;

    	  	    HAL_GPIO_WritePin(MOT_EN_GPIO_Port, MOT_EN_Pin, MOT_ON);			// ���[�^Enable
    		    HAL_GPIO_WritePin(MOT_R_DIR_GPIO_Port, MOT_R_DIR_Pin, MOT_R_FWD);	// �E���[�^�����w�� : �O�i
    		    HAL_GPIO_WritePin(MOT_L_DIR_GPIO_Port, MOT_L_DIR_Pin, MOT_L_FWD);	// �����[�^�����w�� : �O�i
    		    while(1){
    		    	//do{

  					   // �Z���T�l���烉�C����Ԃ�ۑ�
  					   line_val = 0;                             // ���C����ԃ��Z�b�g

  					   sen_val_ml =  sen_val[6];       // �E�Z���T��A/D���l��0-999�ɉ��H
  					   sen_val_mr =  sen_val[2];       // �O�Z���T��A/D���l��0-999�ɉ��H
  					   sen_val_r =  sen_val[3];       // �E�Z���T��A/D���l��0-999�ɉ��H
  					   sen_val_f =  sen_val[4];       // �O�Z���T��A/D���l��0-999�ɉ��H
  					   sen_val_l =  sen_val[5];       // ���Z���T��A/D���l��0-999�ɉ��H
  					   sen_val_gr = sen_val[1];       // ���Z���T��A/D���l��0-999�ɉ��H

  					   R_SEN = (float)(sen_val_r - sen_val_pre_min[3])*1000/(sen_val_pre_max[3] - sen_val_pre_min[3]);
  					   F_SEN = (float)(sen_val_f - sen_val_pre_min[4])*1000/(sen_val_pre_max[4] - sen_val_pre_min[4]);
  					   L_SEN = (float)(sen_val_l - sen_val_pre_min[5])*1000/(sen_val_pre_max[5] - sen_val_pre_min[5]);
  					   L_middle_SEN = (float)(sen_val_ml - sen_val_pre_min[6])*1000/(sen_val_pre_max[6] - sen_val_pre_min[6]);
  					   R_middle_SEN = (float)(sen_val_mr - sen_val_pre_min[2])*1000/(sen_val_pre_max[2] - sen_val_pre_min[2]);

  					   /*if( R_SEN == 1 )     line_val += 1;  // �E�Z���T
  					   if( F_SEN == 1 )     line_val += 2;  // �����Z���T
  					   if( L_SEN == 1 )     line_val += 4;  // ���Z���T*/

  					   //PD_control(L_SEN ,F_SEN ,R_SEN );

  					   if( R_SEN > sen_ref[3] )     line_val += 2;  // �E�Z���T
  					   if( F_SEN > sen_ref[4] )     line_val += 4;  // �����Z���T
  					   if( L_SEN > sen_ref[5] )     line_val += 10;  // ���Z���T
  					   if( R_middle_SEN > sen_ref[2])	line_val += 1;
  					   if( L_middle_SEN > sen_ref[6])	line_val += 11;


  					   /*if( hosei_r_flag == 1){
						   if( R_middle_SEN > sen_ref[2] ){
							   line_val =14;
							   hosei_r_flag = 0;
						   }
					   }
					   else if( hosei_l_flag == 1){
						   if( L_middle_SEN > sen_ref[6] ){
							 line_val = 6;
							 hosei_l_flag = 0;
						   }
					   }*/


  					   if( ext_flag_r == 1 ){
  						   if(((( line_val == 1 || line_val == 3 )|| line_val == 6) || line_val == 7) || line_val == 4 ) {ext_flag_r = 0;}
  					   }
  					   else if( ext_flag_l == 1 ){
  						   if(((( line_val == 11 || line_val == 21 ) || line_val == 25) || line_val == 14) || line_val == 4){ ext_flag_l = 0;}
  					   }

  					   if( ext_flag_r == 1 ){
  						   pre_line_val = 1;
  						   line_val = 0;
  					   }
  					   else if( ext_flag_l == 1 ){
  						   pre_line_val = 11;
  						   line_val = 0;
  					   }
  					   // �Z���T�O�Ή�
  					   if( ( pre_line_val == 1 || pre_line_val == 30 ) && line_val == 0 )
  						   line_val = 30;   // 1�O�̏�Ԃ��E�̂݁C�܂��͉E�Z���T�O�Ή��ŁC�����S�Z���T�������̏ꍇ�F�E�Z���T�O�Ή�
  					   if( ( pre_line_val == 11 || pre_line_val == 31 ) && line_val == 0 )
  						   line_val = 31;   // 1�O�̏�Ԃ����̂݁C�܂��͍��Z���T�O�Ή��ŁC�����S�Z���T�������̏ꍇ�F���Z���T�O�Ή�

  					   LCD_dec_out( 2,  1, target_spd_l, 4 );	// ���[�Z���T�l	�Z�@�~�@�~�@�~�@�~�@�~�@�~
  					   LCD_dec_out( 2,  12, target_spd_r, 4 );	// ���[�Z���T�l	�Z�@�~�@�~�@�~�@�~�@�~�@�~
  					   LCD_dec_out( 1,  7, cross_count_R, 2 );	// ���[�Z���T�l	�Z�@�~�@�~�@�~�@�~�@�~�@�~

  					   // ���C����Ԃ��烂�[�^�ڕW���x�ݒ�
  					   switch( line_val ){

  						   case 1: // line_val = 1 : XXO : �E�Z���T�̂ݔ���
  								   sen_diff_mr = ( sen_val_r - sen_val_mr )-845;
  								   d = (sen_diff_mr - pre_sen_diff_mr)/ pre_id_cnt;
  								   control_r = (sen_diff_mr * Kp) + (d * Kd);
  								   cross_count_R = 0;
  								   target_spd_r = (target_spd + control_r)-650;
  								   target_spd_l = target_spd;
  								   break;

  						   case 2: // line_val = 2 : XOO : �����E�{�E�Z���T����
  								   sen_diff_r = ( sen_val_f - sen_val_r )-845;
  								   d = (sen_diff_r - pre_sen_diff_r)/ pre_id_cnt;
  								   control_r = (sen_diff_r * Kp) + (d * Kd);
  								   cross_count_R = 0;
  								   target_spd_r = (target_spd + control_r)-400;
  								   target_spd_l = target_spd;
  								   line_flag_r = 1;
  								   break;


  						   case 3: // line_val = 3 : XOO : �����E�{�E�Z���T����
  								   sen_diff_mr = ( sen_val_r - sen_val_mr )-845;
  								   d = (sen_diff_mr - pre_sen_diff_mr)/ pre_id_cnt;
  								   control_r = (sen_diff_mr * Kp) + (d * Kd);
  								   cross_count_R = 0;
  								   target_spd_r = (target_spd + control_r)-300;
  								   target_spd_l = target_spd;
  								   break;

  						   case 4: // line_val = 4 : OXX : �����Z���T�̂ݔ���
  								   target_spd_r = target_spd;
  								   target_spd_l = target_spd;
  								   break;

  						   case 5: // line_val = 3 : XOO : �����E�{�E�Z���T����
  								   sen_diff_mr = ( sen_val_r - sen_val_mr )-845;
  								   d = (sen_diff_mr - pre_sen_diff_mr)/ pre_id_cnt;
  								   control_r = (sen_diff_mr * Kp) + (d * Kd);
  								   cross_count_R = 0;
  								   target_spd_r = target_spd + control_r;
  								   target_spd_l = target_spd;
  								   break;

  						   case 6: // line_val = 6 : OOX :����+�����E�Z���T����
  								   sen_diff_r = ( sen_val_f - sen_val_r )-845;
  								   d = (sen_diff_r - pre_sen_diff_r)/ pre_id_cnt;
  								   control_r = (sen_diff_r * Kp) + (d * Kd);
  								   cross_count_R = 0;
  								   target_spd_r = (target_spd + control_r)-200;
  								   target_spd_l = target_spd;
  								   break;

  						   /*case 7: // line_val = 6 : OOX :����+�����E�Z���T����
  								   sen_diff_l = ( sen_val_l - sen_val_f )+845;
  								   d = (sen_diff_l - pre_sen_diff_l)/ pre_id_cnt;
  								   control_l = (sen_diff_l * Kp) + (d * Kd);
  								   cross_count_R = 0;
  								   line_flag_l = 1;
  								   target_spd_r = target_spd;
  								   target_spd_l = target_spd - control_l;
  								   break;*/

  						   case 14: // line_val = 11 : OOX : �����{�������Z���T����
  								   sen_diff_l = ( sen_val_l - sen_val_f )+845;
  								   d = (sen_diff_l - pre_sen_diff_l)/ pre_id_cnt;
  								   control_l = (sen_diff_l * Kp) + (d * Kd);
  								   cross_count_R = 0;
  								   target_spd_r = target_spd;
  								   target_spd_l = (target_spd - control_l)-200;
  								   break;

  						   case 15:// line_val = 12: OOX :���Z���T����
  								   sen_diff_ml = ( sen_val_ml - sen_val_l )+845;
  								   d = (sen_diff_ml - pre_sen_diff_ml)/ pre_id_cnt;
  								   control_l = (sen_diff_ml * Kp) + (d * Kd);
  								   cross_count_R = 0;
  								   hosei_r_flag = 1;
  								   target_spd_r = target_spd;
  								   target_spd_l = target_spd - control_l;
  								   break;


  						   /*case 16: // line_val = 16 : OXX : �����Z���T�̂ݔ���
  								   target_spd_r = target_spd;
  								   target_spd_l = target_spd;
  								   break;

  						   case 28: // line_val = 16 : OXX : �����Z���T�̂ݔ���
  								   target_spd_r = target_spd;
  								   target_spd_l = target_spd;
  								   break;

  						   case 12: // line_val = 16 : OXX : �����Z���T�̂ݔ���
  								   target_spd_r = target_spd;
  								   target_spd_l = target_spd;
  								   break;*/


  						   case 17:// line_val = 19: OOX : �������{���Z���T����
  								   sen_diff_r = ( sen_val_f - sen_val_r )-845;
  								   d = (sen_diff_r - pre_sen_diff_r)/ pre_id_cnt;
  								   control_r = (sen_diff_r * Kp) + (d * Kd);
  								   cross_count_R = 0;
  								   hosei_l_flag = 0;
  								   target_spd_r = target_spd + control_r;
  								   target_spd_l = target_spd;
  								   break;


  						   case 27:// line_val = 19: OOX : �������{���Z���T����
  								   sen_diff_l = ( sen_val_l - sen_val_f )+845;
  								   d = (sen_diff_l - pre_sen_diff_l)/ pre_id_cnt;
  								   control_l = (sen_diff_l * Kp) + (d * Kd);
  								   cross_count_R = 0;
  								   target_spd_r = target_spd;
  								   target_spd_l = target_spd - control_l;
  								   break;


  						   case 10:// line_val = 19: OOX : �������{���Z���T����
  								   sen_diff_l = ( sen_val_l - sen_val_f )+845;
  								   d = (sen_diff_l - pre_sen_diff_l)/ pre_id_cnt;
  								   control_l = (sen_diff_ml * Kp) + (d * Kd);
  								   cross_count_R = 0;
  								   target_spd_r = target_spd;
  								   target_spd_l = (target_spd - control_l)-400;
  								   break;

  						   case 11:// line_val = 12: OOX :���Z���T����
  								   sen_diff_ml = ( sen_val_ml - sen_val_l )+845;
  								   d = (sen_diff_ml - pre_sen_diff_ml)/ pre_id_cnt;
  								   control_l = (sen_diff_ml * Kp) + (d * Kd);
  								   cross_count_R = 0;
  								   target_spd_r = target_spd;
  								   target_spd_l = (target_spd - control_l)-650;
  								   break;


  						   case 13: // line_val = 3 : XOO : �����E�{�E�Z���T����
  								   sen_diff_mr = ( sen_val_r - sen_val_mr )-845;
  								   d = (sen_diff_mr - pre_sen_diff_mr)/ pre_id_cnt;
  								   control_r = (sen_diff_mr * Kp) + (d * Kd);
  								   cross_count_R = 0;
  								   target_spd_r = target_spd + control_r;
  								   target_spd_l = target_spd;
  								   break;


  						   case 21:// line_val = 12: OOX :���Z���T����
  								   sen_diff_ml = ( sen_val_ml - sen_val_l )+845;
  								   d = (sen_diff_ml - pre_sen_diff_ml)/ pre_id_cnt;
  								   control_l = (sen_diff_ml * Kp) + (d * Kd);
  								   cross_count_R = 0;
  								   target_spd_r = target_spd;
  								   target_spd_l = (target_spd - control_l)-300;
  								   break;

  						   case 30: // line_val = 8 : XXX : �E�Z���T�O�Ή�
  								   //target_spd_r = target_spd - ((F_SENP-sen_val_f)*1.5);
  								   ext_flag_r = 1;
  								   cross_count_R = 0;
  								   //trace_gain_r_ext= 0.28;
  								   target_spd_r = 220;//target_spd_r * trace_gain_r_ext;
  								   target_spd_l = 2200;//1.1
  								   break;

  						   case 31: // line_val = 9 : XXX : ���Z���T�O�Ή�
  								   ext_flag_l = 1;
  								   cross_count_R = 0;
  								   //trace_gain_l_ext = 0.28;
  								   target_spd_r = 2200;//1.1
  								   target_spd_l = 220;//target_spd * trace_gain_l_ext;
  								   break;

  						   default:// line_val = 0,5,7 : XXX, OXO, OOO : ��~
  								   target_spd_r = 0;
  								   target_spd_l = 0;
  								   //cross_flg = 1;
  								   break;
  					   }
  					   LCD_dec_out( 2,  1, target_spd_l, 4 );	// ���[�Z���T�l	�Z�@�~�@�~�@�~�@�~�@�~�@�~
  					   LCD_dec_out( 2,  12, target_spd_r, 4 );	// ���[�Z���T�l	�Z�@�~�@�~�@�~�@�~�@�~�@�~
  					   LCD_dec_out( 1,  7, cross_count_R, 5 );	// ���[�Z���T�l	�Z�@�~�@�~�@�~�@�~�@�~�@�~

  						   // ���C����ԕۑ�
  						   pre_line_val = line_val;
  						   pre_sen_diff_r = sen_diff_r;
  						   pre_sen_diff_l = sen_diff_l;
  						   pre_sen_diff_ml = sen_diff_ml;
  						   pre_sen_diff_mr = sen_diff_mr;

  						   if(sen_val[3] > 400 && sen_val[4] > 400 &&sen_val[5] > 400) {
  							   cross_handan = 1;
  							   goal_count = 0;
  						   }

  						   if(s_handan == 0 && sen_val[1] > sen_ref[1] ){
  							   cross_handan = 1;
  							   s_handan = 1;
  						   }

  						   if( goal_count > 2000 ){
  							   cross_handan = 0;
  							   //cross_count_R = 0;
  						   }
  						   if(sen_val[1] > sen_ref[1]){
  							   if( 1 < cross_count_R && cross_count_R < 100 ){
  								   //cross_count_R = 0;
  								   if( cross_handan == 0 && s_handan == 1){
  									   target_spd_r = 1400;
  									   target_spd_l = 1400;
  									   HAL_Delay(300);
  									   target_spd_r = 1000;
  									   target_spd_l = 1000;
  									   HAL_Delay(300);
  									   target_spd_r = 500;
  									   target_spd_l = 500;
  									   HAL_Delay(100);
  									   break;
  								   }
  							   }else{
  								   //cross_count_R = 0;
  							   }
  						   }
  					   if(HAL_GPIO_ReadPin(SW_MID_GPIO_Port, SW_MID_Pin) == SW_ON){
  						   HAL_Delay( SW_WAIT );
  						   break;
  					   }


  				   }//while(  HAL_GPIO_ReadPin(SW_MID_GPIO_Port, SW_MID_Pin) == SW_ON );         // ����SW���͔��f
    				   //HAL_Delay( SW_WAIT );                          // �`���^�����O�h�~
    				   target_spd_r = 0;
    				   target_spd_l = 0;
    				   while( mot_spd_l != 0 || mot_spd_r != 0 );    // ���S��~�҂�
    				   HAL_GPIO_WritePin(MOT_EN_GPIO_Port, MOT_EN_Pin, MOT_OFF);                  // ���[�^Enable OFF

    	  	}
      }
    	else if( mode == 5 ){
      	//------------------------------------------------
      	// Mode 5:���s���[�hthird
      	//------------------------------------------------
      	  if( mode_com == DISP ){  			// DISP�w���̏ꍇ�F���[�h�^�C�g���\��
      	  	  LCD_print( 1, 0, "5: Third Trace" );
      	  	  LCD_print( 2, 0, "                " );
      	  	}else if( mode_com == EXEC ){		// EXEC�w���̏ꍇ�F���[�h���s
      	  		LCD_clear(1);
      	  		third_param();
      	  		sen_ref_all();

      	  		while(1){
      				if(HAL_GPIO_ReadPin(SW_UP_GPIO_Port, SW_UP_Pin) == SW_ON ){// �ESW���͔��f
      					HAL_Delay( SW_WAIT );        // �`���^�����O�h�~
      					break;
      				}
      				if( HAL_GPIO_ReadPin(SW_DOWN_GPIO_Port, SW_DOWN_Pin) == SW_ON ){// ��SW���͔��f
      					HAL_Delay( SW_WAIT );        // �`���^�����O�h�~
      					calibration();
      					//LCD_print( 1, 0, "3: calibration" );
      					HAL_Delay( 1000 );
      					break;
      				}
      			}

      	  		goal_count = 0;
      			s_handan = 0;
      			cross_handan = 0;
      			countdown();
      			cross_count_R = 0;

      	  	    HAL_GPIO_WritePin(MOT_EN_GPIO_Port, MOT_EN_Pin, MOT_ON);			// ���[�^Enable
      		    HAL_GPIO_WritePin(MOT_R_DIR_GPIO_Port, MOT_R_DIR_Pin, MOT_R_FWD);	// �E���[�^�����w�� : �O�i
      		    HAL_GPIO_WritePin(MOT_L_DIR_GPIO_Port, MOT_L_DIR_Pin, MOT_L_FWD);	// �����[�^�����w�� : �O�i
      		    while(1){
      		   //do{

      					   // �Z���T�l���烉�C����Ԃ�ۑ�
      					   line_val = 0;                             // ���C����ԃ��Z�b�g

      					   sen_val_ml =  sen_val[6];       // �E�Z���T��A/D���l��0-999�ɉ��H
  						   sen_val_mr =  sen_val[2];       // �O�Z���T��A/D���l��0-999�ɉ��H
      					   sen_val_r =  sen_val[3];       // �E�Z���T��A/D���l��0-999�ɉ��H
      					   sen_val_f =  sen_val[4];       // �O�Z���T��A/D���l��0-999�ɉ��H
      					   sen_val_l =  sen_val[5];       // ���Z���T��A/D���l��0-999�ɉ��H
      					   sen_val_gr = sen_val[1];       // ���Z���T��A/D���l��0-999�ɉ��H

      					   R_SEN = (float)(sen_val_r - sen_val_pre_min[3])*1000/(sen_val_pre_max[3] - sen_val_pre_min[3]);
      					   F_SEN = (float)(sen_val_f - sen_val_pre_min[4])*1000/(sen_val_pre_max[4] - sen_val_pre_min[4]);
      					   L_SEN = (float)(sen_val_l - sen_val_pre_min[5])*1000/(sen_val_pre_max[5] - sen_val_pre_min[5]);
      					   L_middle_SEN = (float)(sen_val_ml - sen_val_pre_min[6])*1000/(sen_val_pre_max[6] - sen_val_pre_min[6]);
      					   R_middle_SEN = (float)(sen_val_mr - sen_val_pre_min[2])*1000/(sen_val_pre_max[2] - sen_val_pre_min[2]);

      					   /*if( R_SEN == 1 )     line_val += 1;  // �E�Z���T
      					   if( F_SEN == 1 )     line_val += 2;  // �����Z���T
      					   if( L_SEN == 1 )     line_val += 4;  // ���Z���T*/

      					   //PD_control(L_SEN ,F_SEN ,R_SEN );

      					   if( R_SEN > sen_ref[3] )     line_val += 2;  // �E�Z���T
      					   if( F_SEN > sen_ref[4] )     line_val += 4;  // �����Z���T
      					   if( L_SEN > sen_ref[5] )     line_val += 10;  // ���Z���T
      					   if( R_middle_SEN > sen_ref[2])	line_val += 1;
      					   if( L_middle_SEN > sen_ref[6])	line_val += 11;


      					   if( ext_flag_r == 1 ){
  							   if((( line_val == 1 || line_val == 3 )|| line_val == 6) || line_val == 7) {ext_flag_r = 0;}
  						   }
  						   else if( ext_flag_l == 1 ){
  							   if((( line_val == 11 || line_val == 21 ) || line_val == 25) || line_val == 14){ ext_flag_l = 0;}
  						   }

      					   if( ext_flag_r == 1 ){
      						   pre_line_val = 1;
      						   line_val = 0;
      					   }
      					   else if( ext_flag_l == 1 ){
      						   pre_line_val = 11;
      						   line_val = 0;
      					   }
      					   // �Z���T�O�Ή�
      					   if( ( pre_line_val == 1 || pre_line_val == 30 ) && line_val == 0 )
      						   line_val = 30;   // 1�O�̏�Ԃ��E�̂݁C�܂��͉E�Z���T�O�Ή��ŁC�����S�Z���T�������̏ꍇ�F�E�Z���T�O�Ή�
      					   if( ( pre_line_val == 11 || pre_line_val == 31 ) && line_val == 0 )
      						   line_val = 31;   // 1�O�̏�Ԃ����̂݁C�܂��͍��Z���T�O�Ή��ŁC�����S�Z���T�������̏ꍇ�F���Z���T�O�Ή�

      					   LCD_dec_out( 2,  1, target_spd_l, 4 );	// ���[�Z���T�l	�Z�@�~�@�~�@�~�@�~�@�~�@�~
      					   LCD_dec_out( 2,  12, target_spd_r, 4 );	// ���[�Z���T�l	�Z�@�~�@�~�@�~�@�~�@�~�@�~
      					   LCD_dec_out( 1,  7, cross_count_R, 2 );	// ���[�Z���T�l	�Z�@�~�@�~�@�~�@�~�@�~�@�~

      					   // ���C����Ԃ��烂�[�^�ڕW���x�ݒ�
      					   switch( line_val ){

  							   case 1: // line_val = 1 : XXO : �E�Z���T�̂ݔ���
  									   sen_diff_mr = ( sen_val_r - sen_val_mr )-845;
  									   d = (sen_diff_mr - pre_sen_diff_mr)/ pre_id_cnt;
  									   control_r = (sen_diff_mr * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = (target_spd + control_r)-650;
  									   target_spd_l = target_spd;
  									   break;

  							   case 2: // line_val = 2 : XOO : �����E�{�E�Z���T����
  								   	   sen_diff_mr = ( sen_val_r - sen_val_mr )-845;
  									   d = (sen_diff_mr - pre_sen_diff_mr)/ pre_id_cnt;
  									   control_r = (sen_diff_mr * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = (target_spd + control_r)-550;
  									   target_spd_l = target_spd;
  									   break;


  							   case 3: // line_val = 3 : XOO : �����E�{�E�Z���T����
  									   sen_diff_mr = ( sen_val_r - sen_val_mr )-845;
  									   d = (sen_diff_mr - pre_sen_diff_mr)/ pre_id_cnt;
  									   control_r = (sen_diff_mr * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = (target_spd + control_r)-300;
  									   target_spd_l = target_spd;
  									   break;

  							   case 4: // line_val = 4 : OXX : �����Z���T�̂ݔ���
  									   target_spd_r = target_spd;
  									   target_spd_l = target_spd;
  									   break;

  							   case 5: // line_val = 3 : XOO : �����E�{�E�Z���T����
  								   	   sen_diff_r = ( sen_val_f - sen_val_r )-845;
  									   d = (sen_diff_r - pre_sen_diff_r)/ pre_id_cnt;
  									   control_r = (sen_diff_r * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd + control_r;
  									   target_spd_l = target_spd;
  									   break;

  							   case 6: // line_val = 6 : OOX :����+�����E�Z���T����
  									   sen_diff_r = ( sen_val_f - sen_val_r )-845;
  									   d = (sen_diff_r - pre_sen_diff_r)/ pre_id_cnt;
  									   control_r = (sen_diff_r * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd + control_r;
  									   target_spd_l = target_spd;
  									   break;

  							   /*case 7: // line_val = 6 : OOX :����+�����E�Z���T����
  									   sen_diff_l = ( sen_val_l - sen_val_f )+845;
  									   d = (sen_diff_l - pre_sen_diff_l)/ pre_id_cnt;
  									   control_l = (sen_diff_l * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   line_flag_l = 1;
  									   target_spd_r = target_spd;
  									   target_spd_l = target_spd - control_l;
  									   break;*/

  							   case 14: // line_val = 11 : OOX : �����{�������Z���T����
  									   sen_diff_l = ( sen_val_l - sen_val_f )+845;
  									   d = (sen_diff_l - pre_sen_diff_l)/ pre_id_cnt;
  									   control_l = (sen_diff_l * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd;
  									   target_spd_l = target_spd - control_l;
  									   break;

  							   case 15:// line_val = 12: OOX :���Z���T����
  								   	   sen_diff_l = ( sen_val_l - sen_val_f )+845;
  									   d = (sen_diff_l - pre_sen_diff_l)/ pre_id_cnt;
  									   control_l = (sen_diff_l * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd;
  									   target_spd_l = target_spd - control_l;
  									   break;


  							   /*case 16: // line_val = 16 : OXX : �����Z���T�̂ݔ���
  									   target_spd_r = target_spd;
  									   target_spd_l = target_spd;
  									   break;

  							   case 28: // line_val = 16 : OXX : �����Z���T�̂ݔ���
  									   target_spd_r = target_spd;
  									   target_spd_l = target_spd;
  									   break;

  							   case 12: // line_val = 16 : OXX : �����Z���T�̂ݔ���
  									   target_spd_r = target_spd;
  									   target_spd_l = target_spd;
  									   break;*/


  							   case 17:// line_val = 19: OOX : �������{���Z���T����
  									   sen_diff_r = ( sen_val_f - sen_val_r )-845;
  									   d = (sen_diff_r - pre_sen_diff_r)/ pre_id_cnt;
  									   control_r = (sen_diff_r * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd + control_r;
  									   target_spd_l = target_spd;
  									   break;


  							   case 27:// line_val = 19: OOX : �������{���Z���T����
  									   sen_diff_l = ( sen_val_l - sen_val_f )+845;
  									   d = (sen_diff_l - pre_sen_diff_l)/ pre_id_cnt;
  									   control_l = (sen_diff_l * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd;
  									   target_spd_l = target_spd - control_l;
  									   break;


  							   case 10:// line_val = 19: OOX : �������{���Z���T����
  								   	   sen_diff_ml = ( sen_val_ml - sen_val_l )+845;
  									   d = (sen_diff_ml - pre_sen_diff_ml)/ pre_id_cnt;
  									   control_l = (sen_diff_ml * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd;
  									   target_spd_l = (target_spd - control_l)-550;
  									   break;

  							   case 11:// line_val = 12: OOX :���Z���T����
  									   sen_diff_ml = ( sen_val_ml - sen_val_l )+845;
  									   d = (sen_diff_ml - pre_sen_diff_ml)/ pre_id_cnt;
  									   control_l = (sen_diff_ml * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd;
  									   target_spd_l = (target_spd - control_l)-650;
  									   break;


  							   case 13: // line_val = 3 : XOO : �����E�{�E�Z���T����
  									   sen_diff_mr = ( sen_val_r - sen_val_mr )-845;
  									   d = (sen_diff_mr - pre_sen_diff_mr)/ pre_id_cnt;
  									   control_r = (sen_diff_mr * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd + control_r;
  									   target_spd_l = target_spd;
  									   break;


  							   case 21:// line_val = 12: OOX :���Z���T����
  									   sen_diff_ml = ( sen_val_ml - sen_val_l )+845;
  									   d = (sen_diff_ml - pre_sen_diff_ml)/ pre_id_cnt;
  									   control_l = (sen_diff_ml * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd;
  									   target_spd_l = (target_spd - control_l)-300;
  									   break;

  							   case 30: // line_val = 8 : XXX : �E�Z���T�O�Ή�
  									   ext_flag_r = 1;
  									   cross_count_R = 0;
  									   //trace_gain_r_ext= 0.28;
  									   target_spd_r = 225;//target_spd_r * trace_gain_r_ext;
  									   target_spd_l = target_spd;//1.1
  									   break;

  							   case 31: // line_val = 9 : XXX : ���Z���T�O�Ή�
  									   ext_flag_l = 1;
  									   cross_count_R = 0;
  									   //trace_gain_l_ext = 0.28;
  									   target_spd_r = target_spd;//1.1
  									   target_spd_l = 225;//target_spd * trace_gain_l_ext;
  									   break;

  							   default:// line_val = 0,5,7 : XXX, OXO, OOO : ��~
  									   target_spd_r = 0;
  									   target_spd_l = 0;
  									   //cross_flg = 1;
  									   break;
  						   }
      					   LCD_dec_out( 2,  1, target_spd_l, 4 );	// ���[�Z���T�l	�Z�@�~�@�~�@�~�@�~�@�~�@�~
      					   LCD_dec_out( 2,  12, target_spd_r, 4 );	// ���[�Z���T�l	�Z�@�~�@�~�@�~�@�~�@�~�@�~
      					   LCD_dec_out( 1,  7, cross_count_R, 5 );	// ���[�Z���T�l	�Z�@�~�@�~�@�~�@�~�@�~�@�~

      						   // ���C����ԕۑ�
      						   pre_line_val = line_val;
      						   pre_sen_diff_r = sen_diff_r;
      						   pre_sen_diff_l = sen_diff_l;
      						   pre_sen_diff_ml = sen_diff_ml;
      						   pre_sen_diff_mr = sen_diff_mr;


      						   if(s_handan == 0){
  								   target_spd = 2000;
  							   }
  							   else{
  								   target_spd = 2400;
  							   }

      						   if(sen_val[3] > 400 && sen_val[4] > 400 &&sen_val[5] > 400) {
      							   cross_handan = 1;
      							   goal_count = 0;
      						   }

      						   if(s_handan == 0 && sen_val[1] > sen_ref[1] ){
      							   cross_handan = 1;
      							   s_handan = 1;
      						   }

      						   if( goal_count > 2000 ){
      							   cross_handan = 0;
      							   //cross_count_R = 0;
      						   }
      						   if(sen_val[1] > sen_ref[1]){
      							   if( 1 < cross_count_R && cross_count_R < 90 ){
      								   //cross_count_R = 0;
      								   if( cross_handan == 0 && s_handan == 1){
      									   target_spd_r = 1400;
      									   target_spd_l = 1400;
      									   HAL_Delay(300);
      									   target_spd_r = 1000;
      									   target_spd_l = 1000;
      									   HAL_Delay(300);
      									   target_spd_r = 500;
      									   target_spd_l = 500;
      									   HAL_Delay(100);
      									   break;
      								   }
      							   }else{
      								   //cross_count_R = 0;
      							   }
      						   }
      					   if(HAL_GPIO_ReadPin(SW_MID_GPIO_Port, SW_MID_Pin) == SW_ON){
      						   HAL_Delay( SW_WAIT );
      						   break;
      					   }


      				   }//while(  HAL_GPIO_ReadPin(SW_MID_GPIO_Port, SW_MID_Pin) == SW_ON );         // ����SW���͔��f
      				   //HAL_Delay( SW_WAIT );                          // �`���^�����O�h�~
      				   target_spd_r = 0;
      				   target_spd_l = 0;
      				   while( mot_spd_l != 0 || mot_spd_r != 0 );    // ���S��~�҂�
      				   HAL_GPIO_WritePin(MOT_EN_GPIO_Port, MOT_EN_Pin, MOT_OFF);                  // ���[�^Enable OFF

      	  	}
        }
    else if( mode == 6 ){
        	//------------------------------------------------
        	// Mode 6:���s���[�hthird
        	//------------------------------------------------
        	  if( mode_com == DISP ){  			// DISP�w���̏ꍇ�F���[�h�^�C�g���\��
        	  	  LCD_print( 1, 0, "6: Fourth Trace" );
        	  	  LCD_print( 2, 0, "                " );
        	  	}else if( mode_com == EXEC ){		// EXEC�w���̏ꍇ�F���[�h���s
        	  		LCD_clear(1);
        	  		fourth_param();
        	  		sen_ref_all();

        	  		while(1){
        				if(HAL_GPIO_ReadPin(SW_UP_GPIO_Port, SW_UP_Pin) == SW_ON ){// �ESW���͔��f
        					HAL_Delay( SW_WAIT );        // �`���^�����O�h�~
        					break;
        				}
        				if( HAL_GPIO_ReadPin(SW_DOWN_GPIO_Port, SW_DOWN_Pin) == SW_ON ){// ��SW���͔��f
        					HAL_Delay( SW_WAIT );        // �`���^�����O�h�~
        					calibration();
        					//LCD_print( 1, 0, "3: calibration" );
        					HAL_Delay( 1000 );
        					break;
        				}
        			}

        	  		goal_count = 0;
        			s_handan = 0;
        			cross_handan = 0;
        			countdown();
        			cross_count_R = 0;

        	  	    HAL_GPIO_WritePin(MOT_EN_GPIO_Port, MOT_EN_Pin, MOT_ON);			// ���[�^Enable
        		    HAL_GPIO_WritePin(MOT_R_DIR_GPIO_Port, MOT_R_DIR_Pin, MOT_R_FWD);	// �E���[�^�����w�� : �O�i
        		    HAL_GPIO_WritePin(MOT_L_DIR_GPIO_Port, MOT_L_DIR_Pin, MOT_L_FWD);	// �����[�^�����w�� : �O�i
        		    while(1){
        		   //do{

        					   // �Z���T�l���烉�C����Ԃ�ۑ�
        					   line_val = 0;                             // ���C����ԃ��Z�b�g

        					   sen_val_ml =  sen_val[6];       // �E�Z���T��A/D���l��0-999�ɉ��H
    						   sen_val_mr =  sen_val[2];       // �O�Z���T��A/D���l��0-999�ɉ��H
        					   sen_val_r =  sen_val[3];       // �E�Z���T��A/D���l��0-999�ɉ��H
        					   sen_val_f =  sen_val[4];       // �O�Z���T��A/D���l��0-999�ɉ��H
        					   sen_val_l =  sen_val[5];       // ���Z���T��A/D���l��0-999�ɉ��H
        					   sen_val_gr = sen_val[1];       // ���Z���T��A/D���l��0-999�ɉ��H

        					   R_SEN = (float)(sen_val_r - sen_val_pre_min[3])*1000/(sen_val_pre_max[3] - sen_val_pre_min[3]);
        					   F_SEN = (float)(sen_val_f - sen_val_pre_min[4])*1000/(sen_val_pre_max[4] - sen_val_pre_min[4]);
        					   L_SEN = (float)(sen_val_l - sen_val_pre_min[5])*1000/(sen_val_pre_max[5] - sen_val_pre_min[5]);
        					   L_middle_SEN = (float)(sen_val_ml - sen_val_pre_min[6])*1000/(sen_val_pre_max[6] - sen_val_pre_min[6]);
        					   R_middle_SEN = (float)(sen_val_mr - sen_val_pre_min[2])*1000/(sen_val_pre_max[2] - sen_val_pre_min[2]);

        					   /*if( R_SEN == 1 )     line_val += 1;  // �E�Z���T
        					   if( F_SEN == 1 )     line_val += 2;  // �����Z���T
        					   if( L_SEN == 1 )     line_val += 4;  // ���Z���T*/

        					   //PD_control(L_SEN ,F_SEN ,R_SEN );

        					   if( R_SEN > sen_ref[3] )     line_val += 2;  // �E�Z���T
        					   if( F_SEN > sen_ref[4] )     line_val += 4;  // �����Z���T
        					   if( L_SEN > sen_ref[5] )     line_val += 10;  // ���Z���T
        					   if( R_middle_SEN > sen_ref[2])	line_val += 1;
        					   if( L_middle_SEN > sen_ref[6])	line_val += 11;


        					   if( ext_flag_r == 1 ){
  							   if(((( line_val == 1 || line_val == 3 )|| line_val == 6) || line_val == 7) || line_val == 4 ) {ext_flag_r = 0;}
  						   }
  						   else if( ext_flag_l == 1 ){
  							   if(((( line_val == 11 || line_val == 21 ) || line_val == 25) || line_val == 14) || line_val == 4){ ext_flag_l = 0;}
  						   }

        					   if( ext_flag_r == 1 ){
        						   pre_line_val = 1;
        						   line_val = 0;
        					   }
        					   else if( ext_flag_l == 1 ){
        						   pre_line_val = 11;
        						   line_val = 0;
        					   }
        					   if( hosei_r_flag == 1){
								   if( R_middle_SEN > sen_ref[2] ){
									   line_val =14;
								   }
							   }
							   else if( hosei_l_flag == 1){
								   if( L_middle_SEN > sen_ref[6] ){
									 line_val = 6;
								   }
							   }

        					   // �Z���T�O�Ή�
        					   if( ( pre_line_val == 1 || pre_line_val == 30 ) && line_val == 0 )
        						   line_val = 30;   // 1�O�̏�Ԃ��E�̂݁C�܂��͉E�Z���T�O�Ή��ŁC�����S�Z���T�������̏ꍇ�F�E�Z���T�O�Ή�
        					   if( ( pre_line_val == 11 || pre_line_val == 31 ) && line_val == 0 )
        						   line_val = 31;   // 1�O�̏�Ԃ����̂݁C�܂��͍��Z���T�O�Ή��ŁC�����S�Z���T�������̏ꍇ�F���Z���T�O�Ή�

        					   LCD_dec_out( 2,  1, target_spd_l, 4 );	// ���[�Z���T�l	�Z�@�~�@�~�@�~�@�~�@�~�@�~
        					   LCD_dec_out( 2,  12, target_spd_r, 4 );	// ���[�Z���T�l	�Z�@�~�@�~�@�~�@�~�@�~�@�~
        					   LCD_dec_out( 1,  7, cross_count_R, 2 );	// ���[�Z���T�l	�Z�@�~�@�~�@�~�@�~�@�~�@�~

        					   // ���C����Ԃ��烂�[�^�ڕW���x�ݒ�
        					 switch( line_val ){

  							   case 1: // line_val = 1 : XXO : �E�Z���T�̂ݔ���
  									   sen_diff_mr = ( sen_val_r - sen_val_mr )-845;
  									   d = (sen_diff_mr - pre_sen_diff_mr)/ pre_id_cnt;
  									   control_r = (sen_diff_mr * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = (target_spd + control_r)-750;
  									   target_spd_l = target_spd;
  									   break;

  							   case 2: // line_val = 2 : XOO : �����E�{�E�Z���T����
  									   sen_diff_r = ( sen_val_f - sen_val_r )-845;
  									   d = (sen_diff_r - pre_sen_diff_r)/ pre_id_cnt;
  									   control_r = (sen_diff_r * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = (target_spd + control_r)-500;
  									   target_spd_l = target_spd;
  									   line_flag_r = 1;
  									   break;


  							   case 3: // line_val = 3 : XOO : �����E�{�E�Z���T����
  									   sen_diff_mr = ( sen_val_r - sen_val_mr )-845;
  									   d = (sen_diff_mr - pre_sen_diff_mr)/ pre_id_cnt;
  									   control_r = (sen_diff_mr * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = (target_spd + control_r)-350;
  									   target_spd_l = target_spd;
  									   break;

  							   case 4: // line_val = 4 : OXX : �����Z���T�̂ݔ���
  									   target_spd_r = target_spd;
  									   target_spd_l = target_spd;
  									   break;

  							   case 5: // line_val = 3 : XOO : �����E�{�E�Z���T����
  								   	   sen_diff_r = ( sen_val_f - sen_val_r )-845;
  									   d = (sen_diff_r - pre_sen_diff_r)/ pre_id_cnt;
  									   control_r = (sen_diff_r * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd + control_r;
  									   target_spd_l = target_spd;
  									   break;

  							   case 6: // line_val = 6 : OOX :����+�����E�Z���T����
  									   sen_diff_r = ( sen_val_f - sen_val_r )-845;
  									   d = (sen_diff_r - pre_sen_diff_r)/ pre_id_cnt;
  									   control_r = (sen_diff_r * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = (target_spd + control_r)-200;
  									   target_spd_l = target_spd;
  									   break;

  							   /*case 7: // line_val = 6 : OOX :����+�����E�Z���T����
  									   sen_diff_l = ( sen_val_l - sen_val_f )+845;
  									   d = (sen_diff_l - pre_sen_diff_l)/ pre_id_cnt;
  									   control_l = (sen_diff_l * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   line_flag_l = 1;
  									   target_spd_r = target_spd;
  									   target_spd_l = target_spd - control_l;
  									   break;*/

  							   case 14: // line_val = 11 : OOX : �����{�������Z���T����
  									   sen_diff_l = ( sen_val_l - sen_val_f )+845;
  									   d = (sen_diff_l - pre_sen_diff_l)/ pre_id_cnt;
  									   control_l = (sen_diff_l * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd;
  									   target_spd_l = (target_spd - control_l)-200;
  									   break;

  							   case 15:// line_val = 12: OOX :���Z���T����
  								   	   sen_diff_l = ( sen_val_l - sen_val_f )+845;
  									   d = (sen_diff_l - pre_sen_diff_l)/ pre_id_cnt;
  									   control_l = (sen_diff_l * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   hosei_r_flag = 1;
  									   target_spd_r = target_spd;
  									   target_spd_l = target_spd - control_l;
  									   break;


  							   /*case 16: // line_val = 16 : OXX : �����Z���T�̂ݔ���
  									   target_spd_r = target_spd;
  									   target_spd_l = target_spd;
  									   break;

  							   case 28: // line_val = 16 : OXX : �����Z���T�̂ݔ���
  									   target_spd_r = target_spd;
  									   target_spd_l = target_spd;
  									   break;

  							   case 12: // line_val = 16 : OXX : �����Z���T�̂ݔ���
  									   target_spd_r = target_spd;
  									   target_spd_l = target_spd;
  									   break;*/


  							   case 17:// line_val = 19: OOX : �������{���Z���T����
  									   sen_diff_r = ( sen_val_f - sen_val_r )-845;
  									   d = (sen_diff_r - pre_sen_diff_r)/ pre_id_cnt;
  									   control_r = (sen_diff_r * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   hosei_l_flag = 0;
  									   target_spd_r = target_spd + control_r;
  									   target_spd_l = target_spd;
  									   break;


  							   case 27:// line_val = 19: OOX : �������{���Z���T����
  									   sen_diff_l = ( sen_val_l - sen_val_f )+845;
  									   d = (sen_diff_l - pre_sen_diff_l)/ pre_id_cnt;
  									   control_l = (sen_diff_l * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd;
  									   target_spd_l = target_spd - control_l;
  									   break;


  							   case 10:// line_val = 19: OOX : �������{���Z���T����
  									   sen_diff_l = ( sen_val_l - sen_val_f )+845;
  									   d = (sen_diff_l - pre_sen_diff_l)/ pre_id_cnt;
  									   control_l = (sen_diff_ml * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd;
  									   target_spd_l = (target_spd - control_l)-500;
  									   break;

  							   case 11:// line_val = 12: OOX :���Z���T����
  									   sen_diff_ml = ( sen_val_ml - sen_val_l )+845;
  									   d = (sen_diff_ml - pre_sen_diff_ml)/ pre_id_cnt;
  									   control_l = (sen_diff_ml * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd;
  									   target_spd_l = (target_spd - control_l)-750;
  									   break;


  							   case 13: // line_val = 3 : XOO : �����E�{�E�Z���T����
  									   sen_diff_mr = ( sen_val_r - sen_val_mr )-845;
  									   d = (sen_diff_mr - pre_sen_diff_mr)/ pre_id_cnt;
  									   control_r = (sen_diff_mr * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd + control_r;
  									   target_spd_l = target_spd;
  									   break;


  							   case 21:// line_val = 12: OOX :���Z���T����
  									   sen_diff_ml = ( sen_val_ml - sen_val_l )+845;
  									   d = (sen_diff_ml - pre_sen_diff_ml)/ pre_id_cnt;
  									   control_l = (sen_diff_ml * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd;
  									   target_spd_l = (target_spd - control_l)-350;
  									   break;

  							   case 30: // line_val = 8 : XXX : �E�Z���T�O�Ή�
  									   //target_spd_r = target_spd - ((F_SENP-sen_val_f)*1.5);
  									   ext_flag_r = 1;
  									   cross_count_R = 0;
  									   //trace_gain_r_ext= 0.28;
  									   target_spd_r = 100;//target_spd_r * trace_gain_r_ext;
  									   target_spd_l = target_spd;//1.1
  									   break;

  							   case 31: // line_val = 9 : XXX : ���Z���T�O�Ή�
  									   ext_flag_l = 1;
  									   cross_count_R = 0;
  									   //trace_gain_l_ext = 0.28;
  									   target_spd_r = target_spd;//1.1
  									   target_spd_l = 100;//target_spd * trace_gain_l_ext;
  									   break;

  							   default:// line_val = 0,5,7 : XXX, OXO, OOO : ��~
  									   target_spd_r = 0;
  									   target_spd_l = 0;
  									   //cross_flg = 1;
  									   break;
  						   }
        					   LCD_dec_out( 2,  1, target_spd_l, 4 );	// ���[�Z���T�l	�Z�@�~�@�~�@�~�@�~�@�~�@�~
        					   LCD_dec_out( 2,  12, target_spd_r, 4 );	// ���[�Z���T�l	�Z�@�~�@�~�@�~�@�~�@�~�@�~
        					   LCD_dec_out( 1,  7, cross_count_R, 5 );	// ���[�Z���T�l	�Z�@�~�@�~�@�~�@�~�@�~�@�~

        						   // ���C����ԕۑ�
        						   pre_line_val = line_val;
        						   pre_sen_diff_r = sen_diff_r;
        						   pre_sen_diff_l = sen_diff_l;
        						   pre_sen_diff_ml = sen_diff_ml;
        						   pre_sen_diff_mr = sen_diff_mr;

        						   if(s_handan == 0){
  								   target_spd = 2200;
  							   }
  							   else{
  								   target_spd = 2600;
  							   }


        						   if(sen_val[3] > 400 && sen_val[4] > 400 &&sen_val[5] > 400) {
  								   cross_handan = 1;
  								   goal_count = 0;
  							   }

        						   if(s_handan == 0 && sen_val[1] > sen_ref[1] ){
        							   cross_handan = 1;
        							   s_handan = 1;
        						   }

        						   if( goal_count > 2000 ){
        							   cross_handan = 0;
        							   //cross_count_R = 0;
        						   }
        						   if(sen_val[1] > sen_ref[1]){
        							   if( 1 < cross_count_R && cross_count_R < 80 ){
        								   //cross_count_R = 0;
        								   if( cross_handan == 0 && s_handan == 1){
        									   target_spd_r = 1400;
        									   target_spd_l = 1400;
        									   HAL_Delay(300);
        									   target_spd_r = 1000;
        									   target_spd_l = 1000;
        									   HAL_Delay(300);
        									   target_spd_r = 500;
        									   target_spd_l = 500;
        									   HAL_Delay(100);
        									   break;
        								   }
        							   }else{
        								   //cross_count_R = 0;
        							   }
        						   }
        					   if(HAL_GPIO_ReadPin(SW_MID_GPIO_Port, SW_MID_Pin) == SW_ON){
        						   HAL_Delay( SW_WAIT );
        						   break;
        					   }


        				   }//while(  HAL_GPIO_ReadPin(SW_MID_GPIO_Port, SW_MID_Pin) == SW_ON );         // ����SW���͔��f
        				   //HAL_Delay( SW_WAIT );                          // �`���^�����O�h�~
        				   target_spd_r = 0;
        				   target_spd_l = 0;
        				   while( mot_spd_l != 0 || mot_spd_r != 0 );    // ���S��~�҂�
        				   HAL_GPIO_WritePin(MOT_EN_GPIO_Port, MOT_EN_Pin, MOT_OFF);                  // ���[�^Enable OFF

        	  	}
        }
  	  else if( mode == 7 ){
  		//------------------------------------------------
  		// Mode7 :�Z���T�l�擾
  		//------------------------------------------------
  		if( mode_com == DISP ){  			// DISP�w���̏ꍇ�F���[�h�^�C�g���\��
  		  LCD_print( 1, 0, "7: sen_check      " );
  		  LCD_print( 2, 0, "                " );
  		}else if( mode_com == EXEC ){		// EXEC�w���̏ꍇ�F���[�h���s
  			LCD_clear(1);
  			int syutoku_r[256],syutoku_l[256],syutoku_f[256];
  			int syutoku_mr[256],syutoku_ml[256],syutoku_goal[256],syutoku_niji[256];

  			int average_r = 0,average_f = 0,average_l = 0,average_ml =0,average_mr = 0,average_goal = 0,average_niji = 0;
  			int keka_r = 0,keka_f = 0,keka_l = 0,keka_ml =0,keka_mr = 0,keka_goal = 0,keka_niji = 0;

  			for( i=1 ; i<= 256 ; i++){

  			        syutoku_r[i-1] = sen_val[3];
  			        syutoku_l[i-1] = sen_val[5];
  			        syutoku_f[i-1] = sen_val[4];
  			        syutoku_ml[i-1] = sen_val[6];
  			        syutoku_mr[i-1] = sen_val[2];
  			        syutoku_goal[i-1] = sen_val[1];
  			        syutoku_niji[i-1] = sen_val[7];

  			        average_r += syutoku_r[i-1];
  			        average_l += syutoku_l[i-1];
  			        average_f += syutoku_f[i-1];
  			        average_ml += syutoku_ml[i-1];
  			        average_mr += syutoku_mr[i-1];
  			        average_goal += syutoku_goal[i-1];
  			        average_niji += syutoku_niji[i-1];

  			    }
  			    //if( gpio_get( SW_C ) == SW_OFF );         // ����SW���͔��f
  				HAL_Delay( 1000 );

  				keka_r = average_r/256;
  				keka_l = average_l/256;
  				keka_f = average_f/256;
  				keka_ml = average_ml/256;
  				keka_mr = average_mr/256;
  				keka_goal = average_goal/256;
  				keka_niji = average_niji/256;

  				LCD_dec_out( 2, 12, keka_goal, 3 );	// �E�[�Z���T�l	�~�@�~�@�~�@�~�@�~�@�~�@�Z
  				LCD_dec_out( 1, 10, keka_mr, 3 );	// �E2�ڃZ���T�l	�~�@�~�@�~�@�~�@�~�@�Z�@�~
  				LCD_dec_out( 2,  8, keka_r, 3 );	// �����E�Z���T�l	�~�@�~�@�~�@�~�@�Z�@�~�@�~
  				LCD_dec_out( 1,  6, keka_f, 3 );	// �����Z���T�l	�~�@�~�@�~�@�Z�@�~�@�~�@�~
  				LCD_dec_out( 2,  4, keka_l, 3 );	// �������Z���T�l	�~�@�~�@�Z�@�~�@�~�@�~�@�~
  				LCD_dec_out( 1,  2, keka_ml, 3 );	// ��2�ڃZ���T�l	�~�@�Z�@�~�@�~�@�~�@�~�@�~
  				LCD_dec_out( 2,  0, keka_niji, 3 );	// ���[�Z���T�l	�Z�@�~�@�~�@�~�@�~�@�~�@�~

  			    HAL_Delay( 10000 );
  		}
  	}
    else if( mode == 8 ){
  			//------------------------------------------------
  			// Mode 8:���s���[�hthird
  			//------------------------------------------------
  			  if( mode_com == DISP ){  			// DISP�w���̏ꍇ�F���[�h�^�C�g���\��
  				  LCD_print( 1, 0, "8: Ignore Trace" );
  				  LCD_print( 2, 0, "                " );
  				}else if( mode_com == EXEC ){		// EXEC�w���̏ꍇ�F���[�h���s
  					LCD_clear(1);
  					fifth_param();
  					sen_ref_all();

  					while(1){
  						if(HAL_GPIO_ReadPin(SW_UP_GPIO_Port, SW_UP_Pin) == SW_ON ){// �ESW���͔��f
  							HAL_Delay( SW_WAIT );        // �`���^�����O�h�~
  							break;
  						}
  						if( HAL_GPIO_ReadPin(SW_DOWN_GPIO_Port, SW_DOWN_Pin) == SW_ON ){// ��SW���͔��f
  							HAL_Delay( SW_WAIT );        // �`���^�����O�h�~
  							calibration();
  							//LCD_print( 1, 0, "3: calibration" );
  							HAL_Delay( 1000 );
  							break;
  						}
  					}

  					goal_count = 0;
  					s_handan = 0;
  					cross_handan = 0;
  					countdown();
  					cross_count_R = 0;

  					HAL_GPIO_WritePin(MOT_EN_GPIO_Port, MOT_EN_Pin, MOT_ON);			// ���[�^Enable
  					HAL_GPIO_WritePin(MOT_R_DIR_GPIO_Port, MOT_R_DIR_Pin, MOT_R_FWD);	// �E���[�^�����w�� : �O�i
  					HAL_GPIO_WritePin(MOT_L_DIR_GPIO_Port, MOT_L_DIR_Pin, MOT_L_FWD);	// �����[�^�����w�� : �O�i
  					while(1){
  				   //do{

  							   // �Z���T�l���烉�C����Ԃ�ۑ�
  							   line_val = 0;                             // ���C����ԃ��Z�b�g

  							   sen_val_ml =  sen_val[6];       // �E�Z���T��A/D���l��0-999�ɉ��H
  							   sen_val_mr =  sen_val[2];       // �O�Z���T��A/D���l��0-999�ɉ��H
  							   sen_val_r =  sen_val[3];       // �E�Z���T��A/D���l��0-999�ɉ��H
  							   sen_val_f =  sen_val[4];       // �O�Z���T��A/D���l��0-999�ɉ��H
  							   sen_val_l =  sen_val[5];       // ���Z���T��A/D���l��0-999�ɉ��H
  							   sen_val_gr = sen_val[1];       // ���Z���T��A/D���l��0-999�ɉ��H

  							   R_SEN = (float)(sen_val_r - sen_val_pre_min[3])*1000/(sen_val_pre_max[3] - sen_val_pre_min[3]);
  							   F_SEN = (float)(sen_val_f - sen_val_pre_min[4])*1000/(sen_val_pre_max[4] - sen_val_pre_min[4]);
  							   L_SEN = (float)(sen_val_l - sen_val_pre_min[5])*1000/(sen_val_pre_max[5] - sen_val_pre_min[5]);
  							   L_middle_SEN = (float)(sen_val_ml - sen_val_pre_min[6])*1000/(sen_val_pre_max[6] - sen_val_pre_min[6]);
  							   R_middle_SEN = (float)(sen_val_mr - sen_val_pre_min[2])*1000/(sen_val_pre_max[2] - sen_val_pre_min[2]);

  							   /*if( R_SEN == 1 )     line_val += 1;  // �E�Z���T
  							   if( F_SEN == 1 )     line_val += 2;  // �����Z���T
  							   if( L_SEN == 1 )     line_val += 4;  // ���Z���T*/

  							   //PD_control(L_SEN ,F_SEN ,R_SEN );

  							   if( R_SEN > sen_ref[3] )     line_val += 2;  // �E�Z���T
  							   if( F_SEN > sen_ref[4] )     line_val += 4;  // �����Z���T
  							   if( L_SEN > sen_ref[5] )     line_val += 10;  // ���Z���T
  							   if( R_middle_SEN > sen_ref[2])	line_val += 1;
  							   if( L_middle_SEN > sen_ref[6])	line_val += 11;


  							   if( ext_flag_r == 1 ){
  								   if(((( line_val == 1 || line_val == 3 )|| line_val == 6) || line_val == 7) || line_val == 4 ) {ext_flag_r = 0;}
  							   }
  							   else if( ext_flag_l == 1 ){
  								   if(((( line_val == 11 || line_val == 21 ) || line_val == 25) || line_val == 14) || line_val == 4){ ext_flag_l = 0;}
  							   }

  							   if( ext_flag_r == 1 ){
  								   pre_line_val = 1;
  								   line_val = 0;
  							   }
  							   else if( ext_flag_l == 1 ){
  								   pre_line_val = 11;
  								   line_val = 0;
  							   }
  							   // �Z���T�O�Ή�
  							   if( ( pre_line_val == 1 || pre_line_val == 30 ) && line_val == 0 )
  								   line_val = 30;   // 1�O�̏�Ԃ��E�̂݁C�܂��͉E�Z���T�O�Ή��ŁC�����S�Z���T�������̏ꍇ�F�E�Z���T�O�Ή�
  							   if( ( pre_line_val == 11 || pre_line_val == 31 ) && line_val == 0 )
  								   line_val = 31;   // 1�O�̏�Ԃ����̂݁C�܂��͍��Z���T�O�Ή��ŁC�����S�Z���T�������̏ꍇ�F���Z���T�O�Ή�

  							   LCD_dec_out( 2,  1, target_spd_l, 4 );	// ���[�Z���T�l	�Z�@�~�@�~�@�~�@�~�@�~�@�~
  							   LCD_dec_out( 2,  12, target_spd_r, 4 );	// ���[�Z���T�l	�Z�@�~�@�~�@�~�@�~�@�~�@�~
  							   LCD_dec_out( 1,  7, cross_count_R, 2 );	// ���[�Z���T�l	�Z�@�~�@�~�@�~�@�~�@�~�@�~

  							   // ���C����Ԃ��烂�[�^�ڕW���x�ݒ�
  							 switch( line_val ){

  							   case 1: // line_val = 1 : XXO : �E�Z���T�̂ݔ���
  									   sen_diff_mr = ( sen_val_r - sen_val_mr )-845;
  									   d = (sen_diff_mr - pre_sen_diff_mr)/ pre_id_cnt;
  									   control_r = (sen_diff_mr * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = (target_spd + control_r)-550;
  									   target_spd_l = target_spd;
  									   break;

  							   case 2: // line_val = 2 : XOO : �����E�{�E�Z���T����
  									   sen_diff_r = ( sen_val_f - sen_val_r )-845;
  									   d = (sen_diff_r - pre_sen_diff_r)/ pre_id_cnt;
  									   control_r = (sen_diff_r * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd + control_r;
  									   target_spd_l = target_spd;
  									   line_flag_r = 1;
  									   break;


  							   case 3: // line_val = 3 : XOO : �����E�{�E�Z���T����
  									   sen_diff_mr = ( sen_val_r - sen_val_mr )-845;
  									   d = (sen_diff_mr - pre_sen_diff_mr)/ pre_id_cnt;
  									   control_r = (sen_diff_mr * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd + control_r;
  									   target_spd_l = target_spd;
  									   break;

  							   case 4: // line_val = 4 : OXX : �����Z���T�̂ݔ���
  									   target_spd_r = target_spd;
  									   target_spd_l = target_spd;
  									   break;

  							   case 5: // line_val = 3 : XOO : �����E�{�E�Z���T����
  									   sen_diff_mr = ( sen_val_r - sen_val_mr )-845;
  									   d = (sen_diff_mr - pre_sen_diff_mr)/ pre_id_cnt;
  									   control_r = (sen_diff_mr * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd + control_r;
  									   target_spd_l = target_spd;
  									   break;

  							   case 6: // line_val = 6 : OOX :����+�����E�Z���T����
  									   sen_diff_r = ( sen_val_f - sen_val_r )-845;
  									   d = (sen_diff_r - pre_sen_diff_r)/ pre_id_cnt;
  									   control_r = (sen_diff_r * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd + control_r;
  									   target_spd_l = target_spd;
  									   break;

  							   /*case 7: // line_val = 6 : OOX :����+�����E�Z���T����
  									   sen_diff_l = ( sen_val_l - sen_val_f )+845;
  									   d = (sen_diff_l - pre_sen_diff_l)/ pre_id_cnt;
  									   control_l = (sen_diff_l * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   line_flag_l = 1;
  									   target_spd_r = target_spd;
  									   target_spd_l = target_spd - control_l;
  									   break;*/

  							   case 14: // line_val = 11 : OOX : �����{�������Z���T����
  									   sen_diff_l = ( sen_val_l - sen_val_f )+845;
  									   d = (sen_diff_l - pre_sen_diff_l)/ pre_id_cnt;
  									   control_l = (sen_diff_l * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd;
  									   target_spd_l = target_spd - control_l;
  									   break;

  							   case 15:// line_val = 12: OOX :���Z���T����
  									   sen_diff_ml = ( sen_val_ml - sen_val_l )+845;
  									   d = (sen_diff_ml - pre_sen_diff_ml)/ pre_id_cnt;
  									   control_l = (sen_diff_ml * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd;
  									   target_spd_l = (target_spd - control_l)-550;
  									   break;


  							   case 16: // line_val = 16 : OXX : �����Z���T�̂ݔ���
  									   target_spd_r = target_spd;
  									   target_spd_l = target_spd;
  									   break;

  							   case 28: // line_val = 16 : OXX : �����Z���T�̂ݔ���
  									   target_spd_r = target_spd;
  									   target_spd_l = target_spd;
  									   break;

  							   case 12: // line_val = 16 : OXX : �����Z���T�̂ݔ���
  									   target_spd_r = target_spd;
  									   target_spd_l = target_spd;
  									   break;


  							   case 17:// line_val = 19: OOX : �������{���Z���T����
  									   sen_diff_r = ( sen_val_f - sen_val_r )-845;
  									   d = (sen_diff_r - pre_sen_diff_r)/ pre_id_cnt;
  									   control_r = (sen_diff_r * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd + control_r;
  									   target_spd_l = target_spd;
  									   break;


  							   case 27:// line_val = 19: OOX : �������{���Z���T����
  									   sen_diff_l = ( sen_val_l - sen_val_f )+845;
  									   d = (sen_diff_l - pre_sen_diff_l)/ pre_id_cnt;
  									   control_l = (sen_diff_l * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd;
  									   target_spd_l = target_spd - control_l;
  									   break;


  							   case 10:// line_val = 19: OOX : �������{���Z���T����
  									   sen_diff_l = ( sen_val_l - sen_val_f )+845;
  									   d = (sen_diff_l - pre_sen_diff_l)/ pre_id_cnt;
  									   control_l = (sen_diff_ml * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd;
  									   target_spd_l = target_spd - control_l;
  									   break;

  							   case 11:// line_val = 12: OOX :���Z���T����
  									   sen_diff_ml = ( sen_val_ml - sen_val_l )+845;
  									   d = (sen_diff_ml - pre_sen_diff_ml)/ pre_id_cnt;
  									   control_l = (sen_diff_ml * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd;
  									   target_spd_l = (target_spd - control_l)-550;
  									   break;


  							   case 13: // line_val = 3 : XOO : �����E�{�E�Z���T����
  									   sen_diff_mr = ( sen_val_r - sen_val_mr )-845;
  									   d = (sen_diff_mr - pre_sen_diff_mr)/ pre_id_cnt;
  									   control_r = (sen_diff_mr * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd + control_r;
  									   target_spd_l = target_spd;
  									   break;


  							   case 21:// line_val = 12: OOX :���Z���T����
  									   sen_diff_ml = ( sen_val_ml - sen_val_l )+845;
  									   d = (sen_diff_ml - pre_sen_diff_ml)/ pre_id_cnt;
  									   control_l = (sen_diff_ml * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd;
  									   target_spd_l = (target_spd - control_l);
  									   break;

  							   case 30: // line_val = 8 : XXX : �E�Z���T�O�Ή�
  									   //target_spd_r = target_spd - ((F_SENP-sen_val_f)*1.5);
  									   ext_flag_r = 1;
  									   cross_count_R = 0;
  									   //trace_gain_r_ext= 0.28;
  									   target_spd_r = 100;//target_spd_r * trace_gain_r_ext;
  									   target_spd_l = target_spd;//1.1
  									   break;

  							   case 31: // line_val = 9 : XXX : ���Z���T�O�Ή�
  									   ext_flag_l = 1;
  									   cross_count_R = 0;
  									   //trace_gain_l_ext = 0.28;
  									   target_spd_r = target_spd;//1.1
  									   target_spd_l = 100;//target_spd * trace_gain_l_ext;
  									   break;

  							   default:// line_val = 0,5,7 : XXX, OXO, OOO : ��~
  									   target_spd_r = 0;
  									   target_spd_l = 0;
  									   //cross_flg = 1;
  									   break;
  						   }
  							   LCD_dec_out( 2,  1, target_spd_l, 4 );	// ���[�Z���T�l	�Z�@�~�@�~�@�~�@�~�@�~�@�~
  							   LCD_dec_out( 2,  12, target_spd_r, 4 );	// ���[�Z���T�l	�Z�@�~�@�~�@�~�@�~�@�~�@�~
  							   LCD_dec_out( 1,  7, cross_count_R, 5 );	// ���[�Z���T�l	�Z�@�~�@�~�@�~�@�~�@�~�@�~

  								   // ���C����ԕۑ�
  								   pre_line_val = line_val;
  								   pre_sen_diff_r = sen_diff_r;
  								   pre_sen_diff_l = sen_diff_l;
  								   pre_sen_diff_ml = sen_diff_ml;
  								   pre_sen_diff_mr = sen_diff_mr;

  								   if(s_handan == 0){
  									   target_spd = 2200;
  								   }
  								   else{
  									   target_spd = 2700;
  								   }


  								   if(sen_val[3] > 400 && sen_val[4] > 400 &&sen_val[5] > 400) {
  									   cross_handan = 1;
  									   goal_count = 0;
  								   }

  								   if(s_handan == 0 && sen_val[1] > sen_ref[1] ){
  									   cross_handan = 1;
  									   s_handan = 1;
  								   }

  								   if( goal_count > 2000 ){
  									   cross_handan = 0;
  									   //cross_count_R = 0;
  								   }
  								   if(sen_val[1] > sen_ref[1]){
  									   if( 1 < cross_count_R && cross_count_R < 100 ){
  										   //cross_count_R = 0;
  										   if( cross_handan == 0 && s_handan == 1){
  											   target_spd_r = 1400;
  											   target_spd_l = 1400;
  											   HAL_Delay(300);
  											   target_spd_r = 1000;
  											   target_spd_l = 1000;
  											   HAL_Delay(300);
  											   target_spd_r = 500;
  											   target_spd_l = 500;
  											   HAL_Delay(100);
  											   break;
  										   }
  									   }else{
  										   //cross_count_R = 0;
  									   }
  								   }
  							   if(HAL_GPIO_ReadPin(SW_MID_GPIO_Port, SW_MID_Pin) == SW_ON){
  								   HAL_Delay( SW_WAIT );
  								   break;
  							   }


  						   }//while(  HAL_GPIO_ReadPin(SW_MID_GPIO_Port, SW_MID_Pin) == SW_ON );         // ����SW���͔��f
  						   //HAL_Delay( SW_WAIT );                          // �`���^�����O�h�~
  						   target_spd_r = 0;
  						   target_spd_l = 0;
  						   while( mot_spd_l != 0 || mot_spd_r != 0 );    // ���S��~�҂�
  						   HAL_GPIO_WritePin(MOT_EN_GPIO_Port, MOT_EN_Pin, MOT_OFF);                  // ���[�^Enable OFF

  				}
  		}
  }

//==============================================================================
// TIM : Timer������ �R�[���o�b�N�֐�
//==============================================================================
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  // TIM6 : 1KHz = 1ms
  if(htim == &htim6){					// TIM6�Ŋ��荞�ݔ���
		int dev_l = 0, dev_r = 0;
		// �J�E���^�[�X�V
		tick_count++;

		goal_count++;
		if(tick_count == 1000){							// tick_count��1000�ŃN���A
			tick_count = 0;
			time_count++;
		}
		if( sen_val[1] > sen_ref[1] ){
		        cross_count_R++;
		}

		switch( tick_count % 2 ){						// ���荞�ݖ��Ƀ^�X�N�؂�ւ�

			// �^�X�N0 : LCD�X�V
			case 0:	LCD_disp();							// LCD1�����X�V
					break;

			// �^�X�N1 : ���[�^���x����
	        case 1: // �E���[�^����������
					dev_r = target_spd_r - mot_spd_r;	// �ڕW���x�܂ł̕΍�
					if( dev_r > 0 )						// �΍����������x���s��������
						mot_spd_r += MOT_ACC;
					else if( dev_r < 0 )				// �΍����������x���ߏ聁����
						mot_spd_r -= MOT_ACC;

	                // �����[�^����������
					dev_l = target_spd_l - mot_spd_l;	// �ڕW���x�܂ł̕΍�
					if( dev_l > 0 )						// �΍����������x���s��������
						mot_spd_l += MOT_ACC;
					else if( dev_l < 0 )				// �΍����������x���ߏ聁����
						mot_spd_l -= MOT_ACC;

	                // �N�����x�ȉ��̏ꍇ�̏���
	                if( mot_spd_r < MOT_SPD_INIT ){		// �E���[�^�̌��ݑ��x���N�����x�ȉ�
	                    if( target_spd_r == 0 )			// �ڕW���x��0�̎��͌��ݑ��x��0�ɗ��Ƃ�
	                        mot_spd_r = 0;
	                    else							// �ڕW���x��0�ȊO(���������Ƃ��Ă���)���͋N�����x�Ɉ����グ��
	                        mot_spd_r = MOT_SPD_INIT;
	                }
	                if( mot_spd_l < MOT_SPD_INIT ){		// �����[�^�̌��ݑ��x���N�����x�ȉ�
	                    if( target_spd_l == 0 )			// �ڕW���x��0�̎��͌��ݑ��x��0�ɗ��Ƃ�
	                        mot_spd_l = 0;
	                    else							// �ڕW���x��0�ȊO(���������Ƃ��Ă���)���͋N�����x�Ɉ����グ��
	                        mot_spd_l = MOT_SPD_INIT;
	                }

	                // ���E���[�^�w��
	                mot_r_drive();						// ���xmot_spd_r�ŉE���[�^�쓮
	                mot_l_drive();						// ���xmot_spd_l�ō����[�^�쓮
	                break;

	        default:break;
		}
	}
}

//==============================================================================
// ���C���g���[�X�p�@�Z���T�������l
//==============================================================================
void sen_ref_all( void ){
	sen_ref[1] = 500;	// �E�[�Z���T�l	�~�@�~�@�~�@�~�@�~�@�~�@�Z
	sen_ref[2] = 500;	// �E2�ڃZ���T�l	�~�@�~�@�~�@�~�@�~�@�Z�@�~
	sen_ref[3] = 500;	// �����E�Z���T�l	�~�@�~�@�~�@�~�@�Z�@�~�@�~
	sen_ref[4] = 500;	// �����Z���T�l	�~�@�~�@�~�@�Z�@�~�@�~�@�~
	sen_ref[5] = 500;// �������Z���T�l	�~�@�~�@�Z�@�~�@�~�@�~�@�~
	sen_ref[6] = 500;	// ��2�ڃZ���T�l	�~�@�Z�@�~�@�~�@�~�@�~�@�~
	sen_ref[7] = 500;	// ���[�Z���T�l	�Z�@�~�@�~�@�~�@�~�@�~�@�~
}

//==============================================================================
// ���C���g���[�X�p�@�Z���T�������l
//==============================================================================
void set_param( void ){
	    trace_gain_f    = 1.0;          // ���C���g���[�X�p�@����Q�C���F����
	    trace_gain_fr   = 0.9;          // ���C���g���[�X�p�@����Q�C���F�����{�E
	    trace_gain_fl   = 0.9;          // ���C���g���[�X�p�@����Q�C���F�����{��
	    trace_gain_r    = 0.7;          // ���C���g���[�X�p�@����Q�C���F�E
	    trace_gain_l    = 0.7;          // ���C���g���[�X�p�@����Q�C���F��
	    trace_gain_r_ext= 0.33;          // ���C���g���[�X�p�@����Q�C���F�E�Z���T�O
	    trace_gain_l_ext= 0.33;          // ���C���g���[�X�p�@����Q�C���F���Z���T�O
	    target_spd      = 1800;         // ���C���g���[�X�p�@���i���x

	    MOT_ACC = 50;

	    id_flag=0;
	    Kp = 0.45;
	    Kd = 0.0032;
}
//==============================================================================
// ���C���g���[�X�p�@�Z���T�������l
//==============================================================================
void second_param( void ){
	    trace_gain_f    = 1.0;          // ���C���g���[�X�p�@����Q�C���F����
	    trace_gain_fr   = 0.9;          // ���C���g���[�X�p�@����Q�C���F�����{�E
	    trace_gain_fl   = 0.9;          // ���C���g���[�X�p�@����Q�C���F�����{��
	    trace_gain_r    = 0.7;          // ���C���g���[�X�p�@����Q�C���F�E
	    trace_gain_l    = 0.7;          // ���C���g���[�X�p�@����Q�C���F��
	    trace_gain_r_ext= 0.33;          // ���C���g���[�X�p�@����Q�C���F�E�Z���T�O
	    trace_gain_l_ext= 0.33;          // ���C���g���[�X�p�@����Q�C���F���Z���T�O
	    target_spd      = 2200;         // ���C���g���[�X�p�@���i���x

	    MOT_ACC = 60;

	    id_flag=0;
	    Kp = 0.3;
	    Kd = 0.0032;
}
//==============================================================================
// ���C���g���[�X�p�@�Z���T�������l
//==============================================================================
void third_param( void ){
	    trace_gain_f    = 1.0;          // ���C���g���[�X�p�@����Q�C���F����
	    trace_gain_fr   = 0.9;          // ���C���g���[�X�p�@����Q�C���F�����{�E
	    trace_gain_fl   = 0.9;          // ���C���g���[�X�p�@����Q�C���F�����{��
	    trace_gain_r    = 0.7;          // ���C���g���[�X�p�@����Q�C���F�E
	    trace_gain_l    = 0.7;          // ���C���g���[�X�p�@����Q�C���F��
	    trace_gain_r_ext= 0.28;          // ���C���g���[�X�p�@����Q�C���F�E�Z���T�O
	    trace_gain_l_ext= 0.28;          // ���C���g���[�X�p�@����Q�C���F���Z���T�O
	    target_spd      = 2400;         // ���C���g���[�X�p�@���i���x

	    MOT_ACC = 60;

	    id_flag=0;
	    Kp = 0.6;
	    Kd = 0.0028;
}

//==============================================================================
// ���C���g���[�X�p�@�Z���T�������l
//==============================================================================
void fourth_param( void ){
	    trace_gain_f    = 1.0;          // ���C���g���[�X�p�@����Q�C���F����
	    trace_gain_fr   = 0.9;          // ���C���g���[�X�p�@����Q�C���F�����{�E
	    trace_gain_fl   = 0.9;          // ���C���g���[�X�p�@����Q�C���F�����{��
	    trace_gain_r    = 0.7;          // ���C���g���[�X�p�@����Q�C���F�E
	    trace_gain_l    = 0.7;          // ���C���g���[�X�p�@����Q�C���F��
	    trace_gain_r_ext= 0.33;          // ���C���g���[�X�p�@����Q�C���F�E�Z���T�O
	    trace_gain_l_ext= 0.33;          // ���C���g���[�X�p�@����Q�C���F���Z���T�O
	    target_spd      = 2600;         // ���C���g���[�X�p�@���i���x

	    MOT_ACC = 60;

	    id_flag=0;
	    Kp = 0.65;
	    Kd = 0.0032;
}

//==============================================================================
// ���C���g���[�X�p�@�Z���T�������l
//==============================================================================
void fifth_param( void ){
	    trace_gain_f    = 1.0;          // ���C���g���[�X�p�@����Q�C���F����
	    trace_gain_fr   = 0.9;          // ���C���g���[�X�p�@����Q�C���F�����{�E
	    trace_gain_fl   = 0.9;          // ���C���g���[�X�p�@����Q�C���F�����{��
	    trace_gain_r    = 0.7;          // ���C���g���[�X�p�@����Q�C���F�E
	    trace_gain_l    = 0.7;          // ���C���g���[�X�p�@����Q�C���F��
	    trace_gain_r_ext= 0.33;          // ���C���g���[�X�p�@����Q�C���F�E�Z���T�O
	    trace_gain_l_ext= 0.33;          // ���C���g���[�X�p�@����Q�C���F���Z���T�O
	    target_spd      = 2700;         // ���C���g���[�X�p�@���i���x

	    MOT_ACC = 50;

	    id_flag=0;
	    Kp = 0.45;
	    Kd = 0.0028;
}

//==============================================================================
// ADC1&2 : DMA-ADC�R�[���o�b�N�֐�
//==============================================================================
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)		// DMA�I����ɌĂяo�����֐�
{
	if( hadc == &hadc1 ){
		sen_val[1] = adc1_val[0];    	// �E�[�Z���T�l	�~�@�~�@�~�@�~�@�~�@�~�@�Z
		sen_val[2] = adc1_val[1];   	// �E2�ڃZ���T�l	�~�@�~�@�~�@�~�@�~�@�Z�@�~
		sen_val[3] = adc1_val[2];   	// �����E�Z���T�l	�~�@�~�@�~�@�~�@�Z�@�~�@�~
	}
	if( hadc == &hadc2 ){
		sen_val[4] = adc2_val[0];    	// �����Z���T�l	�~�@�~�@�~�@�Z�@�~�@�~�@�~
		sen_val[5] = adc2_val[1];   	// �������Z���T�l	�~�@�~�@�Z�@�~�@�~�@�~�@�~
		sen_val[6] = adc2_val[2];   	// ��2�ڃZ���T�l	�~�@�Z�@�~�@�~�@�~�@�~�@�~
		sen_val[7] = adc2_val[3];   	// ���[�Z���T�l	�Z�@�~�@�~�@�~�@�~�@�~�@�~
	}
}


//==============================================================================
// ���[�^�쓮�֐� : �E���[�^ : ���xmot_spd_r�ŋ쓮
//==============================================================================
void mot_r_drive( void )
{
    if( mot_spd_r <= 0 ){
        // ���[�^��~�w��
    	HAL_TIM_PWM_Stop_IT( &htim3, TIM_CHANNEL_4 );			// �E���[�^OFF
    }else{
        // ���[�^�쓮�w��
    	int pwm_period = 64000 / mot_spd_r - 1;					// prescaler�ɐݒ肷������v�Z
        // ���� = APB1(64MHz) / CounterPeriod(1000) / prescaler(mot_spd) - 1
        __HAL_TIM_SET_PRESCALER(&htim3, pwm_period);			// prescaler��ݒ�
        __HAL_TIM_SET_COMPARE( &htim3, TIM_CHANNEL_4, 500-1);	// duty��ݒ�
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);				// TIM3 �^�C�}�X�^�[�g
    }
}

//==============================================================================
// ���[�^�쓮�֐� : �����[�^ : ���xmot_spd_l�ŋ쓮
//==============================================================================
void mot_l_drive( void )
{
    if( mot_spd_l <= 0 ){
        // ���[�^��~�w��
    	HAL_TIM_PWM_Stop_IT( &htim2, TIM_CHANNEL_3 );			// �����[�^OFF
    }else{
        // ���[�^�쓮�w��
    	int pwm_period = 64000 / mot_spd_l - 1;					// prescaler�ɐݒ肷������v�Z
        // ���� = APB1(64MHz) / CounterPeriod(1000) / prescaler(mot_spd) - 1
        __HAL_TIM_SET_PRESCALER(&htim2, pwm_period);			// prescaler��ݒ�
        __HAL_TIM_SET_COMPARE( &htim2, TIM_CHANNEL_3, 500-1);	// duty��ݒ�
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);				// TIM2 �^�C�}�X�^�[�g
    }
}

//==============================================================================
// �S�[����
//==============================================================================
void finish( void ){
    Beep( TONE_SO, 6, 150 );            // �N����:����6�̃\, 150ms
    Beep(       0, 0, 150 );            // �x��, 150ms
    Beep( TONE_SO, 6, 150 );            // �N����:����6�̃\, 150ms
    Beep(       0, 0,  50 );            // �x��, 50ms
    Beep( TONE_DO, 7, 500 );            // �N����:����7�̃h, 500ms
}

//==============================================================================
// ���C���g���[�X�p�@�Z���T�������l
//==============================================================================
void calibration( void ){
	for( i=0; i<200; i++){
		HAL_Delay(20);

	        //�ŏ��l�̎擾
	        if( sen_val[1] < sen_val_pre_min[1] ){        //�E�Z���T
	        	sen_val_pre_min[1]= sen_val[1] ;
	        }
	        if( sen_val[2] < sen_val_pre_min[2] ){        //�E�Z���T
	        	sen_val_pre_min[2]= sen_val[2] ;
			}
	        if( sen_val[3] < sen_val_pre_min[3] ){        //�E�Z���T
	        	sen_val_pre_min[3]= sen_val[3] ;
			}
	        if( sen_val[4] < sen_val_pre_min[4] ){        //�E�Z���T
	        	sen_val_pre_min[4]= sen_val[4] ;
			}
	        if( sen_val[5] < sen_val_pre_min[5] ){        //�E�Z���T
	        	sen_val_pre_min[5]= sen_val[5] ;
			}
	        if( sen_val[6] < sen_val_pre_min[6] ){        //�E�Z���T
	        	sen_val_pre_min[6]= sen_val[6] ;
			}
	        if( sen_val[7] < sen_val_pre_min[7] ){        //�E�Z���T
	        	sen_val_pre_min[7]= sen_val[7] ;
			}
	        //�ő�l�̎擾
	        if( sen_val[1] > sen_val_pre_max[1] ){        //�E�Z���T
	        	sen_val_pre_max[1]= sen_val[1] ;
	        }
	        if( sen_val[2] > sen_val_pre_max[2] ){        //�E�Z���T
				sen_val_pre_max[2]= sen_val[2] ;
			}
	        if( sen_val[3] > sen_val_pre_max[3] ){        //�E�Z���T
				sen_val_pre_max[3]= sen_val[3] ;
			}
	        if( sen_val[4] > sen_val_pre_max[4] ){        //�E�Z���T
				sen_val_pre_max[4]= sen_val[4] ;
			}
	        if( sen_val[5] > sen_val_pre_max[5] ){        //�E�Z���T
				sen_val_pre_max[5]= sen_val[5] ;
			}
	        if( sen_val[6] > sen_val_pre_max[6] ){        //�E�Z���T
				sen_val_pre_max[6]= sen_val[6] ;
			}
	        if( sen_val[7] > sen_val_pre_max[7] ){        //�E�Z���T
				sen_val_pre_max[7]= sen_val[7] ;
			}
	    }
	 Beep( TONE_DO, 7, 500 );            // �N����:����7�̃h, 500ms
}

//==============================================================================
// �S�[����
//==============================================================================
void countdown( void ){
    Beep( TONE_DO, 7, 500 );            // �N����:����7�̃h, 500ms
}

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
