# ESKF datasets

Put each launch in its own directory under this folder. A launch directory must contain these
four decoded sensor files:

- `BMP581_data.csv`
- `ICM45686_data.csv`
- `MMC5983MA_data.csv`
- `ADXL371_data.csv`

The raw `.FRM` file and a `Calibration/` directory may also be present; the ESKF lab ignores
both. Everything in this directory except this README is ignored by Git.
