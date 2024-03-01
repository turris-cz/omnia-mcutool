#ifndef I2C_IFACE_H
#define I2C_IFACE_H

#include "bits.h"

enum commands_e {
	CMD_GET_STATUS_WORD		= 0x01, /* slave sends status word back */
	CMD_GENERAL_CONTROL		= 0x02,
	CMD_LED_MODE			= 0x03, /* default/user */
	CMD_LED_STATE			= 0x04, /* LED on/off */
	CMD_LED_COLOR			= 0x05, /* LED number + RED + GREEN + BLUE */
	CMD_USER_VOLTAGE		= 0x06,
	CMD_SET_BRIGHTNESS		= 0x07,
	CMD_GET_BRIGHTNESS		= 0x08,
	CMD_GET_RESET			= 0x09,
	CMD_GET_FW_VERSION_APP		= 0x0A, /* 20B git hash number */
	CMD_SET_WATCHDOG_STATE		= 0x0B, /* 0 - disable
						 * 1 - enable / ping
						 * after boot watchdog is started
						 * with 2 minutes timeout
						 */

	/* CMD_WATCHDOG_STATUS		= 0x0C, not implemented anymore */

	CMD_GET_WATCHDOG_STATE		= 0x0D,
	CMD_GET_FW_VERSION_BOOT		= 0x0E, /* 20B git hash number */
	CMD_GET_FW_CHECKSUM		= 0x0F, /* 4B length, 4B checksum */

	/* available if FEATURES_SUPPORTED bit set in status word */
	CMD_GET_FEATURES		= 0x10,

	/* available if EXT_CMD bit set in features */
	CMD_GET_EXT_STATUS_DWORD	= 0x11,
	CMD_EXT_CONTROL			= 0x12,
	CMD_GET_EXT_CONTROL_STATUS	= 0x13,

	/* available if NEW_INT_API bit set in features */
	CMD_GET_INT_AND_CLEAR		= 0x14,
	CMD_GET_INT_MASK		= 0x15,
	CMD_SET_INT_MASK		= 0x16,

	/* available if FLASHING bit set in features */
	CMD_FLASH			= 0x19,

	/* available if WDT_PING bit set in features */
	CMD_SET_WDT_TIMEOUT		= 0x20,
	CMD_GET_WDT_TIMELEFT		= 0x21,

	/* available if POWEROFF_WAKEUP bit set in features */
	CMD_SET_WAKEUP			= 0x22,
	CMD_GET_UPTIME_AND_WAKEUP	= 0x23,
	CMD_POWER_OFF			= 0x24,

	/* available if TRNG bit set in features */
	CMD_TRNG_COLLECT_ENTROPY	= 0x28,

	/* available if CRYPTO bit set in features */
	CMD_CRYPTO_GET_PUBLIC_KEY	= 0x29,
	CMD_CRYPTO_SIGN_MESSAGE		= 0x2A,
	CMD_CRYPTO_COLLECT_SIGNATURE	= 0x2B,

	/* available if BOARD_INFO it set in features */
	CMD_BOARD_INFO_GET		= 0x2C,
	CMD_BOARD_INFO_BURN		= 0x2D,

	/* available only at address 0x2b (led-controller) */
	/* available only if LED_GAMMA_CORRECTION bit set in features */
	CMD_SET_GAMMA_CORRECTION	= 0x30,
	CMD_GET_GAMMA_CORRECTION	= 0x31,
};

enum flashing_commands_e {
	FLASH_CMD_UNLOCK		= 0x01,
	FLASH_CMD_SIZE_AND_CSUM		= 0x02,
	FLASH_CMD_PROGRAM		= 0x03,
	FLASH_CMD_RESET			= 0x04,
};

typedef enum {
	FLASHING_LOCKED = 0,
	FLASHING_EXPECT_SIZE_AND_CSUM,
	FLASHING_EXPECT_PROGRAM,
	FLASHING_BUSY,
	FLASHING_DONE,
	FLASHING_ERR_ERASING,
	FLASHING_ERR_PROGRAMMING,
} flashing_state_t;

enum sts_word_e {
	STS_MCU_TYPE_MASK			= GENMASK(1, 0),
	STS_MCU_TYPE_STM32			= FIELD_PREP(STS_MCU_TYPE_MASK, 0),
	STS_MCU_TYPE_GD32			= FIELD_PREP(STS_MCU_TYPE_MASK, 1),
	STS_MCU_TYPE_MKL			= FIELD_PREP(STS_MCU_TYPE_MASK, 2),
#define STS_MCU_TYPE CONCAT(STS_MCU_TYPE_, MCU_TYPE)
	STS_FEATURES_SUPPORTED			= BIT(2),
	STS_USER_REGULATOR_NOT_SUPPORTED	= BIT(3),
	STS_CARD_DET				= BIT(4),
	STS_MSATA_IND				= BIT(5),
	STS_USB30_OVC				= BIT(6),
	STS_USB31_OVC				= BIT(7),
	STS_USB30_PWRON				= BIT(8),
	STS_USB31_PWRON				= BIT(9),
	STS_ENABLE_4V5				= BIT(10),
	STS_BUTTON_MODE				= BIT(11),
	STS_BUTTON_PRESSED			= BIT(12),
	STS_BUTTON_COUNTER_MASK			= GENMASK(15, 13)
};

enum ctl_byte_e {
	CTL_LIGHT_RST		= BIT(0),
	CTL_HARD_RST		= BIT(1),
	/*CTL_RESERVED		= BIT(2),*/
	CTL_USB30_PWRON		= BIT(3),
	CTL_USB31_PWRON		= BIT(4),
	CTL_ENABLE_4V5		= BIT(5),
	CTL_BUTTON_MODE		= BIT(6),
	CTL_BOOTLOADER		= BIT(7)
};

enum features_e {
	FEAT_PERIPH_MCU			= BIT(0),
	FEAT_EXT_CMDS			= BIT(1),
	FEAT_WDT_PING			= BIT(2),
	FEAT_LED_STATE_EXT_MASK		= GENMASK(4, 3),
	FEAT_LED_STATE_EXT		= FIELD_PREP(FEAT_LED_STATE_EXT_MASK, 1),
	FEAT_LED_STATE_EXT_V32		= FIELD_PREP(FEAT_LED_STATE_EXT_MASK, 2),
	FEAT_LED_GAMMA_CORRECTION	= BIT(5),
	FEAT_NEW_INT_API		= BIT(6),
	FEAT_BOOTLOADER			= BIT(7),
	FEAT_FLASHING			= BIT(8),
	FEAT_NEW_MESSAGE_API		= BIT(9),
	FEAT_BRIGHTNESS_INT		= BIT(10),
	FEAT_POWEROFF_WAKEUP		= BIT(11),
	FEAT_CAN_OLD_MESSAGE_API	= BIT(12),
	FEAT_TRNG			= BIT(13),
	FEAT_CRYPTO			= BIT(14),
	FEAT_BOARD_INFO			= BIT(15),
};

enum ext_sts_dword_e {
	EXT_STS_SFP_nDET		= BIT(0),
	EXT_STS_LED_STATES_MASK		= GENMASK(31, 12),
	EXT_STS_WLAN0_MSATA_LED		= BIT(12),
	EXT_STS_WLAN1_LED		= BIT(13),
	EXT_STS_WLAN2_LED		= BIT(14),
	EXT_STS_WPAN0_LED		= BIT(15),
	EXT_STS_WPAN1_LED		= BIT(16),
	EXT_STS_WPAN2_LED		= BIT(17),
	EXT_STS_WAN_LED0		= BIT(18),
	EXT_STS_WAN_LED1		= BIT(19),
	EXT_STS_LAN0_LED0		= BIT(20),
	EXT_STS_LAN0_LED1		= BIT(21),
	EXT_STS_LAN1_LED0		= BIT(22),
	EXT_STS_LAN1_LED1		= BIT(23),
	EXT_STS_LAN2_LED0		= BIT(24),
	EXT_STS_LAN2_LED1		= BIT(25),
	EXT_STS_LAN3_LED0		= BIT(26),
	EXT_STS_LAN3_LED1		= BIT(27),
	EXT_STS_LAN4_LED0		= BIT(28),
	EXT_STS_LAN4_LED1		= BIT(29),
	EXT_STS_LAN5_LED0		= BIT(30),
	EXT_STS_LAN5_LED1		= BIT(31),
};

enum ext_ctl_e {
	EXT_CTL_nRES_MMC		= BIT(0),
	EXT_CTL_nRES_LAN		= BIT(1),
	EXT_CTL_nRES_PHY		= BIT(2),
	EXT_CTL_nPERST0			= BIT(3),
	EXT_CTL_nPERST1			= BIT(4),
	EXT_CTL_nPERST2			= BIT(5),
	EXT_CTL_PHY_SFP			= BIT(6),
	EXT_CTL_PHY_SFP_AUTO		= BIT(7),
	EXT_CTL_nVHV_CTRL		= BIT(8),
};

enum cmd_poweroff_e {
	CMD_POWER_OFF_POWERON_BUTTON	= BIT(0),
	CMD_POWER_OFF_MAGIC		= 0xdead,
};

#endif /* I2C_IFACE_H */
