# Plan: Export Radiometric Calibration Data to CSV

This plan details the steps to export radiometric calibration data, including reference reflectances, observed DN values, and calculated coefficients, into a CSV file. This will follow the structure implied by the `dnmulti.md` template.

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
- Open a CSV file named `radiometric_report.csv` in the specified output directory for writing. Use standard CSV formatting with commas as delimiters and periods for decimal points.
- Write a header row to the CSV file. The header should be derived from the `dnmulti.md` template, adapted for CSV:
  `Filename,Band,Target 56%,Target 36%,Target 12%,Target 3%,DN 56%,DN 36%,DN 12%,DN 3%,Slope (a),Intercept (b)`
- Iterate through each `pair` (representing a group) in the `allGroups` map.
- For each `GroupData` (`pair.second`), if its `coeffs.valid` is true:
    - **Handle RGB Images:** If `coeffs.isRGB` is true:
        - Write a row for the Red channel using `coeffs.filename`, "Red", `coeffs.targets_r`, `coeffs.dns_r`, `coeffs.a_r`, and `coeffs.b_r`.
        - Write a row for the Green channel using `coeffs.filename`, "Green", `coeffs.targets_g`, `coeffs.dns_g`, `coeffs.a_g`, and `coeffs.b_g`.
        - Write a row for the Blue channel using `coeffs.filename`, "Blue", `coeffs.targets_b`, `coeffs.dns_b`, `coeffs.a_b`, and `coeffs.b_b`.
    - **Handle Multispectral Images:** If `coeffs.isRGB` is false:
        - Write a row for the single band using `coeffs.filename`, `coeffs.bandName`, `coeffs.targets`, `coeffs.dns`, `coeffs.a`, and `coeffs.b`.
- Ensure that floating-point numbers are formatted with sufficient precision for CSV output.


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
    - Header matches the expected format.
    - Data for reference reflectances, DN values, slope, and intercept are accurately recorded.
    - Multispectral and RGB data are handled correctly, with appropriate band names and per-channel details for RGB.
    - Decimal points are consistently represented (e.g., using '.').
- **Data Integrity:** Compare exported values with expected values from the tool's internal calculations.
