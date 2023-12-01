/*
 *	カーネル作成用ダミータスクのヘッダファイル
 */

/*
 *  ターゲット依存の定義
 */
#include "target_test.h"

/*
 *  タスクの定義
 */
#define UROS_PRIORITY	10		/* メインタスクの優先度 */
#define	UROS_STACK_SIZE	(4096U)	/* メインタスクのスタックサイズ */

#define	UROS_PORTID		4		/* 文字入力するシリアルポートID */

static uint8_t img_smile[1][25] = {
  {
    0b0000000, 0b0000000, 0b0000000, 0b0000000, 0b0000000,
    0b0000000, 0b1100100, 0b0000000, 0b1100100, 0b0000000,
    0b0000000, 0b0000000, 0b0000000, 0b0000000, 0b0000000, 
    0b1100100, 0b0000000, 0b0000000, 0b0000000, 0b1100100,
    0b0000000, 0b1100100, 0b1100100, 0b1100100, 0b0000000,
  },
};

static uint8_t img_sad[1][25] = {
  {
    0b0000000, 0b0000000, 0b0000000, 0b0000000, 0b0000000,
    0b0000000, 0b1100100, 0b0000000, 0b1100100, 0b0000000,
    0b0000000, 0b0000000, 0b0000000, 0b0000000, 0b0000000, 
    0b0000000, 0b1100100, 0b1100100, 0b1100100, 0b0000000,
    0b1100100, 0b0000000, 0b0000000, 0b0000000, 0b1100100,
  },
};

/*Timer-callback cycle*/
#define SP_ROS_STA_CYC 100

/*opening message*/
#define DISABLE_OPENING false

/*
 *  関数のプロトタイプ宣言
 */
#ifndef TOPPERS_MACRO_ONLY

#ifdef __cplusplus
extern "C" {
#endif

extern void	uros_task(intptr_t exinf);

#ifdef __cplusplus
}
#endif

#endif /* TOPPERS_MACRO_ONLY */
