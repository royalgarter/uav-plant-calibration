# Plan: Export Radiometric Calibration Data to CSV

This plan details the steps to export radiometric calibration data, including reference reflectances, observed DN values, and calculated coefficients, into a CSV file.

## Configuration File

The reference reflectance values are loaded from `radiometric_reference.csv` in the project root. This file contains the reference reflectance values for each band:

```csv
# Radiometric Reference Reflectances
# Format: band,patch56,patch36,patch12,patch3
# Bands: 0R=RGB_Red, 0G=RGB_Green, 0B=RGB_Blue, 1=Blue, 2=Green, 3=Red, 4=RedEdge, 5=NIR
band,patch56,patch36,patch12,patch3
5,0.5647,0.3582,0.1148,0.0272
4,0.5618,0.3666,0.1191,0.0267
3,0.5599,0.3740,0.1228,0.0262
2,0.5567,0.3835,0.1256,0.0256
1,0.5500704789,0.390806116,0.126676427,0.02539390723
0R,0.5581634717,0.3790530195,0.124637386,0.02591108772
0G,0.5554092039,0.3851975973,0.1255882691,0.02554883719
0B,0.5500704789,0.390806116,0.126676427,0.02539390723
```

If the config file is not found, default values are used.

## Proposed Changes

### 1. Enhance `RadioCoeffs` Struct

Modify the `RadioCoeffs` struct in `src/calib.cc` to store additional information required for the CSV export:
- Add `string filename;` to store the name of the image file used for calibration.
- Add `string bandName;` to store the identified band name (e.g., "NIR", "Red", "Green", "Blue").
- Add `vector<double> targets;` and `vector<double> dns;` for single-band (multispectral) images, storing the reference reflectances and observed DN values for the four calibration patches.
- Add `vector<double> targets_r, targets_g, targets_b;` and `vector<double> dns_r, dns_g, dns_b;` for RGB images, storing per-channel reference reflectances and observed DN values for the four calibration patches.

### 2. Update `getRadiometricCoeffs` Function

Modify the `getRadiometricCoeffs` function in `src/calib.cc` to populate the new fields in the `RadioCoeffs` struct:
- Store the `filename` passed to the function into `coeffs.filename`.
- Determine and store the `bandName` (e.g., "NIR", "Red", "Green", "Blue") into `coeffs.bandName` based on the filename's last character, similar to how it's done for `targets`.
- Store the collected `targets` and `dns` vectors into `coeffs.targets` and `coeffs.dns` respectively for single-band images.
- For RGB images, store the per-channel `targets_r`, `targets_g`, `targets_b` and `dns_r`, `dns_g`, `dns_b` into the corresponding fields in the `coeffs` object.

### 3. Implement CSV Export Function

Create a new function, e.g., `exportRadiometricCsv`, in `src/calib.cc`. This function will:
- Accept the output directory path (`const string& outPath`) and the `allGroups` map (`const map<string, GroupData>& allGroups`) as input.
- Open a CSV file named `radiometric_report.csv` in the specified output directory for writing.
- Write a header row with 34 columns:
  - `Filename` - The image filename
  - `DN_B1_P1` through `DN_B5_P4` - 20 columns for multispectral bands 1-5, 4 patches each
  - `DN_RGB_R_P1` through `DN_RGB_B_P4` - 12 columns for RGB channels (R, G, B), 4 patches each
  - `Slope (a)` and `Intercept (b)` - Calibration coefficients
- Iterate through each group in `allGroups` and write one row per group with all DN values.
- If a band's DN values are not available, output `0` as placeholder.


### 4. Integrate CSV Export in `main()`

In the `main` function of `src/calib.cc`:
- Locate the end of the "RADIOMETRIC CALIBRATION PHASE" block (after the loop that calls `getRadiometricCoeffs`).
- Before the start of "Step 3: Process all groups", add a call to the new `exportRadiometricCsv` function.
- Pass the output directory path (`outDir`) and a const reference to the `allGroups` map to this function.
- Ensure the `doRadio` flag is checked, so the export only occurs if radiometric calibration was enabled.

## Verification & Testing

- **Compilation:** Ensure the code compiles without errors after adding the new struct members and function.
- **CSV File Generation:** Run the calibration with the `--radio` flag enabled and verify that `radiometric_report.csv` is created in the output directory.
- **CSV Content:** Check the generated CSV file for correctness:
  - Header has 34 columns: Filename, 20 DN columns for bands 1-5, 12 DN columns for RGB, plus Slope and Intercept.
  - Data for all bands (1-5) and RGB channels are accurately recorded.
  - Missing band data is filled with zeros.
  - Decimal points are consistently represented (e.g., using '.').
- **Config File:** Verify that reference values are correctly loaded from `radiometric_reference.csv`.
- **Data Integrity:** Compare exported values with expected values from the tool's internal calculations.
