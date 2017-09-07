/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 *
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2004-2010 by Embedded and Real-Time Systems Laboratory
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *
 *  上記著作権者は，以下の(1)～(4)の条件を満たす場合に限り，本ソフトウェ
 *  ア（本ソフトウェアを改変したものを含む．以下同じ）を使用・複製・改
 *  変・再配布（以下，利用と呼ぶ）することを無償で許諾する．
 *  (1) 本ソフトウェアをソースコードの形で利用する場合には，上記の著作
 *      権表示，この利用条件および下記の無保証規定が，そのままの形でソー
 *      スコード中に含まれていること．
 *  (2) 本ソフトウェアを，ライブラリ形式など，他のソフトウェア開発に使
 *      用できる形で再配布する場合には，再配布に伴うドキュメント（利用
 *      者マニュアルなど）に，上記の著作権表示，この利用条件および下記
 *      の無保証規定を掲載すること．
 *  (3) 本ソフトウェアを，機器に組み込むなど，他のソフトウェア開発に使
 *      用できない形で再配布する場合には，次のいずれかの条件を満たすこ
 *      と．
 *    (a) 再配布に伴うドキュメント（利用者マニュアルなど）に，上記の著
 *        作権表示，この利用条件および下記の無保証規定を掲載すること．
 *    (b) 再配布の形態を，別に定める方法によって，TOPPERSプロジェクトに
 *        報告すること．
 *  (4) 本ソフトウェアの利用により直接的または間接的に生じるいかなる損
 *      害からも，上記著作権者およびTOPPERSプロジェクトを免責すること．
 *      また，本ソフトウェアのユーザまたはエンドユーザからのいかなる理
 *      由に基づく請求からも，上記著作権者およびTOPPERSプロジェクトを
 *      免責すること．
 * 
 *  本ソフトウェアは，無保証で提供されているものである．上記著作権者お
 *  よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
 *  に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
 *  アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
 *  の責任を負わない．
 *
 *  $Id: sample1.h 2416 2012-09-07 08:06:20Z ertl-hiro $
 */

/*
 *  ターゲット依存の定義
 */
#include "target_test.h"

/*
 *  各タスクの優先度の定義
 */

#define MAIN_PRIORITY	5		/* メインタスクの優先度 */
								/* HIGH_PRIORITYより高くすること */

#define HIGH_PRIORITY	9		/* 並行実行されるタスクの優先度 */
#define MID_PRIORITY	10
#define LOW_PRIORITY	11

/*
 *  ターゲットに依存する可能性のある定数の定義
 */

#ifndef TASK_PORTID
#define	TASK_PORTID		1			/* 文字入力するシリアルポートID */
#endif /* TASK_PORTID */

#ifndef STACK_SIZE
#define	STACK_SIZE		4096		/* タスクのスタックサイズ */
#endif /* STACK_SIZE */

#ifndef LOOP_REF
#define LOOP_REF		ULONG_C(1000000)	/* 速度計測用のループ回数 */
#endif /* LOOP_REF */

#define COVER_MOTOR 0
#define SPLAY_MOTOR 1

#define PUSH 0
#define NOTPUSH 1

/*
 * フラグのビット割り当て
 */
#define SEAT_ON       0x01U		// 着座検出
#define SEAT_OFF      0x02U     // 起立検出
#define DETECTED      0x04U     // 人検出
#define UNDETECTED    0x08U     // 人未検出
#define SPLAY_ON      0x10U
#define SPLAY_OFF     0x20U

#define CALIB_FONT (EV3_FONT_SMALL)
#define CALIB_FONT_WIDTH (6/*TODO: magic number*/)
#define CALIB_FONT_HEIGHT (8/*TODO: magic number*/)

#define NOZZLE_DEGREE 180
#define NOZZLE_OFF_DEGREE -180
#define NOZZLE_SPEED 50 //persent
#define SPLAY_DEGREE 90
#define SPLAY_CLOSE_DEGREE -90
#define SPLAY_OPEN_SPEED 50 //persent
#define SPLAY_CLOSE_SPEED 50 //persent
#define FLASH_MOTOR_SPEED 50 //persent
#define FLASH_DEGREE 90
#define FLASH_STOP_MOTOR_SPEED 50 //persent
#define FLASH_STOP_DEGREE -90
#define FLASH_TIME 3   //水を流す時間

#define COVER_OPEN_DEGREE 270
#define COVER_CLOSE_DEGREE -270
#define COVER_OPEN_SPEED 10 //persent
#define COVER_CLOSE_SPEED 10 //persent

#define COVER_CLOSE_TIME 5 // 「着座してない」検出から自動的にふたが閉まるまでの時間

//#define HUMAN_DETECT 1000 //超音波センサーで人検出する距離
#define HUMAN_DETECT 10 //超音波センサーで人検出する距離	//TODO 要調整
#define UNDETECTED_VAL 0
#define DETECTED_VAL 1

#define SEATED_MIN 10 //着座とみなす色
#define SEATED_MAX 50 //着座とみなす色

#define FLASH_SOUND NOTE_E4  //流水音

#define SPLAY_TIMER 10  //スプレーが自動的に止まるまでの時間
#define FLASH_TIMER 10


/*
 *  関数のプロトタイプ宣言
 */
#ifndef TOPPERS_MACRO_ONLY

extern void	main_task(intptr_t exinf);

 //extern void sessalet_main_task(intptr_t exinf);
 //extern void seated_task(intptr_t exinf);
 //extern void flash_task(intptr_t exinf);
 //extern void splay_task(intptr_t exinf);
 extern void human_check_cyc(intptr_t exinf);
 //extern void seated_check_task(intptr_t exinf);
 //extern void flush_check_cyc(intptr_t exinf);


#ifdef CPUEXC1
extern void	cpuexc_handler(void *p_excinf);
#endif /* CPUEXC1 */

#endif /* TOPPERS_MACRO_ONLY */
