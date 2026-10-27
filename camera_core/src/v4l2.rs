//! The V4L2 control ids and values the UVC cameras drive, from
//! `<linux/v4l2-controls.h>`. They are stable kernel ABI, spelled out here as
//! plain constants.

const CID_BASE: u32 = 0x0098_0900;
const CID_CAMERA_CLASS_BASE: u32 = 0x009a_0900;

pub const CID_BRIGHTNESS: u32 = CID_BASE;
pub const CID_CONTRAST: u32 = CID_BASE + 1;
pub const CID_AUTO_WHITE_BALANCE: u32 = CID_BASE + 12;
pub const CID_GAIN: u32 = CID_BASE + 19;
pub const CID_WHITE_BALANCE_TEMPERATURE: u32 = CID_BASE + 26;
pub const CID_EXPOSURE_AUTO: u32 = CID_CAMERA_CLASS_BASE + 1;
/// Absolute exposure, in 100 us units.
pub const CID_EXPOSURE_ABSOLUTE: u32 = CID_CAMERA_CLASS_BASE + 2;

/// `V4L2_EXPOSURE_AUTO` of `enum v4l2_exposure_auto_type`, a value of
/// [`CID_EXPOSURE_AUTO`].
pub const EXPOSURE_AUTO: i64 = 0;
/// `V4L2_EXPOSURE_MANUAL` of `enum v4l2_exposure_auto_type`.
pub const EXPOSURE_MANUAL: i64 = 1;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cids_match_the_kernel_abi() {
        // Values from <linux/v4l2-controls.h>.
        assert_eq!(CID_BRIGHTNESS, 0x0098_0900);
        assert_eq!(CID_CONTRAST, 0x0098_0901);
        assert_eq!(CID_AUTO_WHITE_BALANCE, 0x0098_090c);
        assert_eq!(CID_GAIN, 0x0098_0913);
        assert_eq!(CID_WHITE_BALANCE_TEMPERATURE, 0x0098_091a);
        assert_eq!(CID_EXPOSURE_AUTO, 0x009a_0901);
        assert_eq!(CID_EXPOSURE_ABSOLUTE, 0x009a_0902);
        assert_eq!((EXPOSURE_AUTO, EXPOSURE_MANUAL), (0, 1));
    }
}
