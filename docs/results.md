# Project requirements and reported results

## Requirement source

The supplied APSC 200/293 P2 Winter 2026 brief specifies a 60 mm radius path; +108°, −198°, and +90° relative moves; 13.5 s and 9 s processing dwells; assembly diameter below 160 mm; and contact restricted to the lower 10 mm of the femoral head. The transmission speed ratio must differ by at least 25% when rotational transfer is used. The course provides the electronic hardware and limits additional construction to allowed reclaimed/common materials.

## Reported demonstration

The team report, dated March 29, 2026, describes successful dwell timing, secure support of the femoral-head model, and use of reclaimed materials. Its conclusion reports station deviations within roughly an 8° range and states that the ≤2° objective was not achieved at every station. Table 8 reports smaller errors at stations 1 and 3 and a higher error at station 2. These descriptions support a working sequence with residual angular-accuracy limitations, rather than a single uniform accuracy figure.

Shenghan separately identified `anti_stall_encoder_sign_flip` as the successful running program. Its exact saved parameters are documented in `controller.md`.

## Motor testing

The supplied graph compares 50% and 80% duty. Speed decreases as torque rises; the 50% series reaches approximately zero speed at the highest plotted load, while the 80% series retains positive speed. The chart labels its error bars as minimum/average/maximum. No raw spreadsheet or plotting script was supplied, so the plot is reproduced without synthesizing measurement data.

## Geometry

A 4.0 cm driver and 7.5 cm driven cap give an ideal output-to-motor speed ratio of 4/7.5 ≈ 0.5333 and a motor-to-output angle factor of 1.875. Ideal torque multiplication is 1.875 before losses. These are geometric relationships; the elastic belt introduces additional uncertainty in physical output positioning.

## Source records

- APSC 200/293 P2 project brief, Winter 2026: motion and timing requirements, constraints.
- Team 02, Section 203, P2 Final Project Report, March 29, 2026: Sections 3.4–3.6, Table 8, Section 5.1, motor-test plot and mechanical calculations.
- Shenghan Gao's project portfolio and supplied code archive.

The full PDFs are excluded from this public package to avoid republishing student identifiers and unnecessary course/report material.
