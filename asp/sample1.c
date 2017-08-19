
#include <kernel.h>
#include <t_syslog.h>
#include <t_stdlib.h>
#include "syssvc/serial.h"
#include "syssvc/syslog.h"
#include "kernel_cfg.h"
#include "sample1.h"

#include "stm32f4xx_hal.h"
#include "stm32f401xe.h"
#include "../arch/arm_m_gcc/stm32f4xx_stm32cube/stm32f4xx_HAL_Driver/inc/stm32f4xx_hal_tim.h"
#include "../arch/arm_m_gcc/stm32f4xx_stm32cube/stm32f4xx_HAL_Driver/inc/stm32f4xx_hal_i2c.h"


#include "rpr0521.h"
#include "rpr0521_driver.h"
#include "rpr-0521rs.h"


TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef *hi2c1_p = &hi2c1;

static void MX_TIM2_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
static void MX_I2C1_Init(void);

TIM_MasterConfigTypeDef sMasterConfig;
TIM_OC_InitTypeDef sConfigOC;


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
 *  ループ回数
 */
ulong_t	task_loop;		/* タスク内でのループ回数 */
ulong_t	tex_loop;		/* 例外処理ルーチン内でのループ回数 */


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

void user_pwm_setvalue(uint16_t value)
{
//    TIM_OC_InitTypeDef sConfigOC;

//    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = value;
//    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
//    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
//    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
}

void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
 // __GPIOC_CLK_ENABLE();
 // __GPIOH_CLK_ENABLE();
 // __GPIOA_CLK_ENABLE();	// AはTOPPERS内でenable済み
  __GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PA10 */
//  GPIO_InitStruct.Pin = GPIO_PIN_10;
//  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
//  GPIO_InitStruct.Pull = GPIO_PULLUP;
//  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 */
//  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
//  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/*  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
*/
  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

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
//    _Error_Handler(__FILE__, __LINE__);
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
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */
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
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }

}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
{

  if(hi2c->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();

    /**I2C1 GPIO Configuration
    PB8     ------> I2C1_SCL
    PB9     ------> I2C1_SDA
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }

}


void main_task(intptr_t exinf)
{
	ER_UINT	ercd;
#ifdef TOPPERS_SUPPORT_GET_UTM
	SYSUTM	utime1, utime2;
#endif /* TOPPERS_SUPPORT_GET_UTM */

	SVC_PERROR(syslog_msk_log(LOG_UPTO(LOG_INFO), LOG_UPTO(LOG_EMERG)));
	syslog(LOG_NOTICE, "Sample program starts (exinf = %d).", (int_t) exinf);

	/*
	 *  シリアルポートの初期化
	 *
	 *  システムログタスクと同じシリアルポートを使う場合など，シリアル
	 *  ポートがオープン済みの場合にはここでE_OBJエラーになるが，支障は
	 *  ない．
	 */
	ercd = serial_opn_por(TASK_PORTID);
	if (ercd < 0 && MERCD(ercd) != E_OBJ) {
		syslog(LOG_ERROR, "%s (%d) reported by `serial_opn_por'.",
									itron_strerror(ercd), SERCD(ercd));
	}
	SVC_PERROR(serial_ctl_por(TASK_PORTID,
							(IOCTL_CRLF | IOCTL_FCSND | IOCTL_FCRCV)));

	tex_loop = task_loop / 4;

//	HAL_Init();

	  MX_GPIO_Init();
	  MX_TIM2_Init();
	  MX_I2C1_Init();
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);


	  int cnt = 0;
	  int pwm_value = 0;
	  int step = 0;
//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
///	  I2C_Dev_Search(); //動いたよ！！

//	    I2CCommonBegin();	//必ずtrueを返してるだけ。不要？

	    rpr0521_wait_until_found(&hi2c1);
///	    rpr0521_wait_until_found();
	    syslog(LOG_INFO, "\nSensor found.\n\r");
	    rpr0521_initial_setup(&hi2c1);
///	    rpr0521_initial_setup();
	    tslp_tsk(1000);

	while(1){
//		syslog(LOG_INFO, "count:%d", cnt);
//		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
//		cnt++;
//		if(cnt > 5){
//			break;
//		}
//		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);
//		  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
/* 人検知センサーチェック
		  GPIO_PinState state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7);
			syslog(LOG_INFO, "state:%d", state);
		  if(state == GPIO_PIN_RESET){
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
		  }else{
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
		  }
*/

		rpr0521_print_one_value(&hi2c1);
///		rpr0521_print_one_value();
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);	//in1
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);	//in2

//		user_pwm_setvalue(pwm_value);
//		  if(pwm_value == 0) step = 200;
//		  if(pwm_value > 1001) step = -200;
//		  pwm_value += step;

//		  user_pwm_setvalue(100);
//		  tslp_tsk(1000);
//		  user_pwm_setvalue(999);

//			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
//			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);

			tslp_tsk(1000);
//			  tslp_tsk(500);

	}

	syslog(LOG_NOTICE, "Sample program ends.");
	SVC_PERROR(ext_ker());
	assert(0);
}

/* TIM2 init function */

static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 3;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
//    _Error_Handler(__FILE__, __LINE__);
		syslog(LOG_ERROR, "HAL_TIM_Base_Init != HAL_OK");
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
//    _Error_Handler(__FILE__, __LINE__);
		syslog(LOG_ERROR, "HAL_TIM_ConfigClockSource != HAL_OK");
  }

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
//    _Error_Handler(__FILE__, __LINE__);
		syslog(LOG_ERROR, "HAL_TIM_PWM_Init != HAL_OK");
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
//    _Error_Handler(__FILE__, __LINE__);
		syslog(LOG_ERROR, "HAL_TIMEx_MasterConfigSynchronization != HAL_OK");
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
//    _Error_Handler(__FILE__, __LINE__);
		syslog(LOG_ERROR, "HAL_TIM_PWM_ConfigChannel != HAL_OK");
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
//    _Error_Handler(__FILE__, __LINE__);
		syslog(LOG_ERROR, "HAL_TIM_PWM_ConfigChannel2 != HAL_OK");
  }

  HAL_TIM_MspPostInit(&htim2);

}


void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{

  if(htim_base->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspInit 0 */

  /* USER CODE END TIM2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();
  /* USER CODE BEGIN TIM2_MspInit 1 */

  /* USER CODE END TIM2_MspInit 1 */
  }

}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(htim->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM3_MspPostInit 0 */

  /* USER CODE END TIM3_MspPostInit 0 */

    /**TIM3 GPIO Configuration
    PB4     ------> TIM3_CH1
    PB5     ------> TIM3_CH2
    */
//	GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM3_MspPostInit 1 */

  /* USER CODE END TIM3_MspPostInit 1 */
  }


}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{

  if(htim_base->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM3_MspDeInit 0 */

  /* USER CODE END TIM3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();
  /* USER CODE BEGIN TIM3_MspDeInit 1 */

  /* USER CODE END TIM3_MspDeInit 1 */
  }
}

