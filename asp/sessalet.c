

#include <kernel.h>
#include <t_syslog.h>
#include <t_stdlib.h>
#include "syssvc/serial.h"
#include "syssvc/syslog.h"
#include "kernel_cfg.h"
#include "sessalet.h"

#include "stm32f4xx_hal.h"
#include "stm32f401xe.h"
#include "../arch/arm_m_gcc/stm32f4xx_stm32cube/stm32f4xx_HAL_Driver/inc/stm32f4xx_hal_tim.h"
#include "../arch/arm_m_gcc/stm32f4xx_stm32cube/stm32f4xx_HAL_Driver/inc/stm32f4xx_hal_i2c.h"


#include "rpr0521.h"
#include "rpr0521_driver.h"

//TIM2
TIM_HandleTypeDef htim2;
//I2C1
I2C_HandleTypeDef hi2c1;

static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

TIM_MasterConfigTypeDef sMasterConfig;
TIM_OC_InitTypeDef sConfigOC;

#define     MAX_SPEED           100      /* motor()  set: 0 to 100   */

typedef enum Human {
	undetected = 0,
	detected
}Human;

typedef enum Splay{
	stop = 0,
	splay
}Splay;

#define cover_close 0
#define cover_open 1

static Splay splay_mode = stop;	//使用
static int cover_mode = cover_close;
static int cover_mode2 = 0;
static Human human_mode = undetected;
//TODO イベントフラグを使うように変更する
//static FLGPTN flg_sessalet = 0;
//static FLGPTN flg_splay = 0;
//static FLGPTN flg_seated = 0;
static int flg_sessalet = 0;
static int flg_splay = 0;
static int flg_seated = 0;

static int timeout_flg = 0;
static int timeout_counter = 0;

/*
 *  サービスコールのエラーのログ出力
 */
Inline void
svc_perror(const char *file, int_t line, const char *expr, ER ercd)
{
	if (ercd < 0) {
		t_perror(LOG_ERROR, file, line, expr, ercd);
	}
}

#define	SVC_PERROR(expr)	svc_perror(__FILE__, __LINE__, #expr, (expr))

/*
 *  並行実行されるタスクへのメッセージ領域
 */
char	message[3];

/*
 *  CPU例外ハンドラ
 */
#ifdef CPUEXC1

void
cpuexc_handler(void *p_excinf)
{
	ID		tskid;

	syslog(LOG_NOTICE, "CPU exception handler (p_excinf = %08p).", p_excinf);
	if (sns_ctx() != true) {
		syslog(LOG_WARNING,
					"sns_ctx() is not true in CPU exception handler.");
	}
	if (sns_dpn() != true) {
		syslog(LOG_WARNING,
					"sns_dpn() is not true in CPU exception handler.");
	}
	syslog(LOG_INFO, "sns_loc = %d sns_dsp = %d sns_tex = %d",
									sns_loc(), sns_dsp(), sns_tex());
	syslog(LOG_INFO, "xsns_dpn = %d xsns_xpn = %d",
									xsns_dpn(p_excinf), xsns_xpn(p_excinf));

	if (xsns_xpn(p_excinf)) {
		syslog(LOG_NOTICE, "Sample program ends with exception.");
		SVC_PERROR(ext_ker());
		assert(0);
	}

	SVC_PERROR(iget_tid(&tskid));
	SVC_PERROR(iras_tex(tskid, 0x8000U));
}

#endif /* CPUEXC1 */

void motor_setvalue(int motor, int direction, uint16_t value)
{
	uint32_t channel;
	if(motor == COVER_MOTOR){
		channel = TIM_CHANNEL_1;
		if(value == 0){
		    sConfigOC.Pulse = value;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);	//in1 1のin2
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0);	//in1 1のin1
		}else if(direction == 0){
		    sConfigOC.Pulse = value * 10;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);	//in1 1のin2
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0);	//in1 1のin1
		}else{
		    sConfigOC.Pulse = value * -10;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);	//in1 1のin2
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);	//in1 1のin1
		}
	}else if(motor == SPLAY_MOTOR){
		channel = TIM_CHANNEL_2;
		if(value == 0){
		    sConfigOC.Pulse = value;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 0);	//in1 2のin1
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 0);	//in2 2のin2
		}else if(direction == 0){
		    sConfigOC.Pulse = value * 10;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 1);	//in1 2のin1
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 0);	//in2 2のin2
		}else{
		    sConfigOC.Pulse = value * -10;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 0);	//in1 2のin1
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1);	//in2 2のin2
		}
	}
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, channel);
    HAL_TIM_PWM_Start(&htim2, channel);
}

void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
 // __GPIOC_CLK_ENABLE();
 // __GPIOH_CLK_ENABLE();
 // __GPIOA_CLK_ENABLE();	// AはTOPPERS内でenable済み
  __GPIOB_CLK_ENABLE();

//TODO   GPIO_InitStruct.Speed = GPIO_SPEED_FAST; ??

  /*Configure GPIO pins : PA5 PA6 PA7 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin   = GPIO_PIN_7;
  GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin   = GPIO_PIN_6 | GPIO_PIN_10;
  GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_10, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

}

void I2C_Dev_Search(){

    uint8_t FindNum=0;
    uint8_t FindDev[128];

    syslog(LOG_NOTICE,"*** I2C Device Search Start! ***\n\r");
    int i = 0;
    for(i=0; i<0xff;i=i+2){

        uint8_t res=HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)i<<1,(uint8_t*)0x00,0,50);
        if(res==HAL_OK){
            FindDev[FindNum]=i;
            FindNum++;
            syslog(LOG_NOTICE,"[0x%X] \t",i);
        }
        else{
        	syslog(LOG_NOTICE,"0x%X \t",i);
        }

        if((i+2)%10==0)syslog(LOG_NOTICE,"\n");
		tslp_tsk(1);
    }
    syslog(LOG_NOTICE,"\nDevice Found: %d \n",FindNum);
    for(i=0; i<FindNum; i++){
    	syslog(LOG_NOTICE,"Device No. %d  Address: 0x%X (0x%X)\n",i+1,FindDev[i],FindDev[i]>>1);
    }
    syslog(LOG_NOTICE,"*** I2C Device Search Finished! ***\n\r");
	tslp_tsk(100);

}

static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_ENABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
		syslog(LOG_INFO, "err: HAL_I2C_Init != HAL_OK");
  }else{
		syslog(LOG_INFO, "ok: HAL_I2C_Init == HAL_OK status:%u", hi2c1.State);
  }

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(hi2c->Instance==I2C1)
  {
//	    __HAL_RCC_I2C1_CLK_ENABLE();	//TODO こっちに移動した cf.http://memo.tank.jp/archives/12040

    /**I2C1 GPIO Configuration
    PB8     ------> I2C1_SCL
    PB9     ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
  }

}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
{

  if(hi2c->Instance==I2C1)
  {
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();

    /**I2C1 GPIO Configuration
    PB8     ------> I2C1_SCL
    PB9     ------> I2C1_SDA
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);
  }

}

int seated_check(){
	int result = UNDETECTED_VAL;
	float als_val;
	unsigned short ps_val;
	unsigned char rc = get_psalsval(&hi2c1, &ps_val, &als_val);
	if (rc == 0) {
		if (ps_val > HUMAN_DETECT) {
			result = DETECTED_VAL;
		}
	}
	return result;
}


int human_check2(){
	GPIO_PinState state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6);
	if(state == GPIO_PIN_SET){
		return DETECTED_VAL;
	}else{
		return UNDETECTED_VAL;
	}
}

void human_check_cyc(intptr_t unused) {
	int result = human_check2();
	switch(result){
		case DETECTED_VAL:
			syslog(LOG_ERROR, "human detected");
			flg_sessalet = DETECTED;
			human_mode = detected;
			break;
		case UNDETECTED_VAL:
			syslog(LOG_ERROR, "human undetected");
			flg_sessalet = UNDETECTED;
			human_mode = detected;
			break;
		default:
			break;
	}

}


// Flashタスク流水音の開始と指定時間後の停止
void flash_task(intptr_t unused) {
	syslog(LOG_ERROR, "--- flash task---");
//	  action.startFlash(FLASH_TIME);
}

#define REVERSE 1
#define FOWARD 0

void coverclose(){
	motor_setvalue(COVER_MOTOR, FOWARD, 100);
	dly_tsk(1000);
	motor_setvalue(COVER_MOTOR, FOWARD, 0);
	dly_tsk(3000);	// フタが閉まるまで待つ（パカパカするの防止）
}

void coveropen(){
	motor_setvalue(COVER_MOTOR, REVERSE, 100);
	dly_tsk(3000);
	motor_setvalue(COVER_MOTOR, REVERSE, 0);
	dly_tsk(1000);	// 少し待つ
}

void spraystart(){
	motor_setvalue(SPLAY_MOTOR, FOWARD, 100);
	dly_tsk(1000);	// 少し待つ
}

void spraystop(){
	motor_setvalue(SPLAY_MOTOR, FOWARD, 0);
	dly_tsk(100);
}


void initialize(){
	/*
	 *  シリアルポートの初期化
	 *
	 *  システムログタスクと同じシリアルポートを使う場合など，シリアル
	 *  ポートがオープン済みの場合にはここでE_OBJエラーになるが，支障は
	 *  ない．
	 */
	ER_UINT	ercd;
	ercd = serial_opn_por(TASK_PORTID);
	if (ercd < 0 && MERCD(ercd) != E_OBJ) {
		syslog(LOG_ERROR, "%s (%d) reported by `serial_opn_por'.",
									itron_strerror(ercd), SERCD(ercd));
	}
	SVC_PERROR(serial_ctl_por(TASK_PORTID,
							(IOCTL_CRLF | IOCTL_FCSND | IOCTL_FCRCV)));

	//	HAL_Init();	// TOPPERSの中でやってる

	MX_GPIO_Init();
	MX_TIM2_Init();
	MX_I2C1_Init();
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

	///	  I2C_Dev_Search(); //動いたよ！！

	// 近接センサーの初期化
	rpr0521_wait_until_found(&hi2c1);
	syslog(LOG_INFO, "\nSensor found.\n\r");
	rpr0521_initial_setup(&hi2c1);

	tslp_tsk(1000);
}

void main_task(intptr_t exinf)
{
	ER_UINT	ercd;
#ifdef TOPPERS_SUPPORT_GET_UTM
	SYSUTM	utime1, utime2;
#endif /* TOPPERS_SUPPORT_GET_UTM */

	SVC_PERROR(syslog_msk_log(LOG_UPTO(LOG_INFO), LOG_UPTO(LOG_EMERG)));
	syslog(LOG_NOTICE, "Sample program starts (exinf = %d).", (int_t) exinf);

	cover_mode2 = 0;	//TODO test
	initialize();

	int cnt = 0;
	int pwm_value = 0;
	int step = 0;
	int splay_value = 0;
	char msg[100];

	// 初期化が終わったら人検知を始める
	sta_cyc(HUMAN_CHECK_CYC);

	while(1) {

		syslog(LOG_ERROR, "loop");
	    dly_tsk(5000);

		  // 人検出か着座検出を待つ。5秒間たったらふたを閉める
		timeout_flg = 0;
		timeout_counter = 0;
		while(timeout_flg == 0){	//タイムアウトがきたら1に変わっているはず
			tslp_tsk(1000);
			syslog(LOG_ERROR, "-- mode:sessalet= %u, cover= %d", flg_sessalet, cover_mode2);
			if((flg_sessalet == DETECTED) && (cover_mode2 == cover_open)){
				int result = seated_check();
				switch(result){
					case DETECTED_VAL:
						syslog(LOG_ERROR, "-- seated");
						flg_seated = SEAT_ON;
						break;
					case UNDETECTED_VAL:
						syslog(LOG_ERROR, "-- not seated");
						flg_seated = SEAT_OFF;
						break;
					default:
						break;
				}
			}
			if((flg_sessalet == DETECTED) && (flg_seated == SEAT_ON)){//着座検出 -> 着座状態へ遷移
				memset(msg, 0x00, sizeof(msg));
				sprintf(msg, "--- act seated task, from main task ---\r\n");
				syslog(LOG_ERROR, msg, sizeof(msg));
				syslog(LOG_ERROR, "seated mode");
				int splay_flg = 0;
				while(1){	//着座中のループ
					int result = seated_check();
					switch(result){
						case DETECTED_VAL:
							syslog(LOG_ERROR, "-- seated");
							flg_seated = SEAT_ON;
							break;
						case UNDETECTED_VAL:
							syslog(LOG_ERROR, "-- not seated");
							flg_seated = SEAT_OFF;
							if(splay_flg == 1){
								syslog(LOG_ERROR, "**splay stop");
								splay_flg = 0;
								spraystop();
							}
							break;
						default:
							break;
					}
					if(flg_seated == SEAT_OFF){
						syslog(LOG_ERROR, "UNDETECTED exit seated loop");
						break;	//exit 着座中のループ
					}

					GPIO_PinState pushbutton = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7);
					  if(pushbutton == 1){
						  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
//				            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
							syslog(LOG_ERROR, "**spray not push");
					}else{	//スプレーボタン
						  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
//				            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
							syslog(LOG_NOTICE, "state : reset.");
						if(splay_flg == 0){
							syslog(LOG_ERROR, "**splay start");
							//flush
							spraystart();
							splay_flg = 1;
						}else{
							syslog(LOG_ERROR, "**splay stop");
							splay_flg = 0;
							spraystop();
						}
					}
					dly_tsk(500);
				}
//  		  act_tsk(SEATED_TASK);
///  			  slp_tsk();

			}else if(flg_sessalet == DETECTED){	//人検出

				if(cover_mode2 == cover_close){
					memset(msg, 0x00, sizeof(msg));
					sprintf(msg, "--- cover open ---\r\n");
					syslog(LOG_ERROR, msg, sizeof(msg));
					coveropen();
					cover_mode2 = cover_open;
					dly_tsk(1000);
	//				act_tsk(SEATED_TASK);
	//   			  slp_tsk();
				}
			}else if(flg_sessalet == UNDETECTED){
				if(cover_mode2 == cover_open){
					timeout_counter++;
					if(timeout_counter > 5){
						timeout_flg = 1;
					}
				}
			}else if(flg_sessalet == SEAT_OFF){

			}
		}
		if(timeout_flg == 1){
			timeout_flg = 0;
			timeout_counter = 0;
			memset(msg, 0x00, sizeof(msg));
			sprintf(msg, "--- timeout cover close ---\r\n");
			syslog(LOG_ERROR, msg, sizeof(msg));
			coverclose();
			cover_mode2 = cover_close;
		}
		dly_tsk(500);
	}
	SVC_PERROR(ext_ker());
	assert(0);
}

/* TIM2 init function */

static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;

  htim2.Init.Period = 0xFFFF;
  htim2.Init.Prescaler = 0;
  htim2.Init.ClockDivision = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
	  // TODO エラー処理
		syslog(LOG_ERROR, "HAL_TIM_Base_Init != HAL_OK");
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
	  // TODO エラー処理
		syslog(LOG_ERROR, "HAL_TIM_ConfigClockSource != HAL_OK");
  }

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
	  // TODO エラー処理
		syslog(LOG_ERROR, "HAL_TIM_PWM_Init != HAL_OK");
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
	  // TODO エラー処理
		syslog(LOG_ERROR, "HAL_TIMEx_MasterConfigSynchronization != HAL_OK");
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
	  // TODO エラー処理
		syslog(LOG_ERROR, "HAL_TIM_PWM_ConfigChannel != HAL_OK");
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
	  // TODO エラー処理
		syslog(LOG_ERROR, "HAL_TIM_PWM_ConfigChannel2 != HAL_OK");
  }

  HAL_TIM_MspPostInit(&htim2);

}


void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{

  if(htim_base->Instance==TIM2)
  {
    /* Peripheral clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();
  }

}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{

	  GPIO_InitTypeDef GPIO_InitStruct;
	  if(htim->Instance==TIM2)
	  {
	    /**TIM2 GPIO Configuration
	    PA0-WKUP     ------> TIM2_CH1
	    PA1     ------> TIM2_CH2
	    */
	    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
	    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
	    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  }
}

