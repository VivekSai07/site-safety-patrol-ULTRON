# Detection-world public sources (simulation-only, rights review)

Status: desk review on 2026-11-28 from public pages and a public preview only.
No assets from the external candidate sources below were imported into the
world, uploaded or published as part of this review, and no private data was
sent. The default world uses the existing team scene and Unitree model; the
earlier teammate hall is optional and supplied locally. Their provenance and
license boundaries are described in
[detection-world usage](DETECTION_WORLD_USAGE.md). No real held-out evaluation
has occurred; nothing here is detector performance. This is not legal advice
or a grant of rights. Oracle detections stay oracle.

The public [ConSLAM preview](https://github.com/mac137/ConSLAM) shows an
unfinished indoor site with stacked materials and strong side illumination.
The test hall adds abstract material stacks and an oblique light as approximate
test features; it does not reproduce the site or import its imagery.

## Ladder image datasets

| Source | Reported license / verification status | Restrictions to verify | Fit |
|---|---|---|---|
| Open Images V7, class `/m/012w5l,Ladder` (`storage.googleapis.com/openimages/v7/oidv7-class-descriptions-boxable.csv`) | Google reports CC BY 4.0 annotations and lists images as CC BY 2.0, without guaranteeing each image's license (`storage.googleapis.com/openimages/web/factsfigures_v7.html`, Licenses). | Verify each image's rights and retain required attribution. V7 reports exhaustive boxes for positive image-level classes, except group cases; unverified classes cannot be assumed absent. | Generic ladder detection reference. Not construction-specific, no damage labels. |
| Roboflow Universe “People and Ladders” (`universe.roboflow.com/roboflow-universe-projects/people-and-ladders`) | A search snippet reports CC BY 4.0; the JavaScript-rendered dataset page was not verified. | Terms and upstream image rights are unverified; do not assume permission. | Ladder/person boxes; scene reference. |
| Roboflow Universe “Ladder Dectation” by ALadder (`universe.roboflow.com/aladder/ladder-dectation`) | A search snippet reports CC BY 4.0; the dataset page was not verified. | Terms and upstream image rights are unverified; do not assume permission. | Small community set; quality unknown. |

Roboflow licenses are uploader-declared, not independent rights clearance.

For a tiny local real-image smoke check, four individually attributed Wikimedia
Commons ladder photographs were collected with approximate model-assisted
boxes. Local provenance records each source, creator, stated license and
modification; these records do not independently establish ownership or
privacy clearance. The annotations have not been independently reviewed by a
human, contain no confirmed inspection-label annotations or verified negative
images, and may overlap the pretrained model's data. They are not a held-out
benchmark.

## Construction/indoor datasets and 3D assets

| Source | Reported license / verification status | Restrictions to verify | Fit |
|---|---|---|---|
| ConSLAM (University of Cambridge; `repository.cam.ac.uk/handle/1810/345700`, GitHub `mac137/ConSLAM`) | The repository describes the item as all rights reserved; the GitHub README states an academic-use restriction. | Do not infer a general permission, including for academic use; verify any actual grant and do not bundle. | Real construction-site SLAM reference (images, lidar, IMU); no ladder labels. |
| Poly Haven models/HDRIs/textures (`polyhaven.com/license`) | The site reports CC0 for its assets. | Verify each asset page and applicable website terms. | Candidate simulated props/materials; few construction items. |
| Objaverse (`huggingface.co/datasets/allenai/objaverse`) | The dataset page reports ODC-By 1.0 for the collection; individual object licenses vary, including CC BY, CC BY-NC, CC BY-SA and CC0. | Check each object's metadata and source. Apply noncommercial terms to the intended use, regardless of academic affiliation; retain required attribution and track share-alike terms. | Ladder/site props only after per-asset filtering. |

Not reviewed (gaps): Sketchfab individual models, ScanNet/Matterport3D/HM3D
(access agreements), ConPR and other construction sets. Unverified until their
original terms are read.

## Nesseler data (separately rights-unverified)

Nesseler photos are local event material. No license was verified and none is
inferred. They are outside the approved publication scope: keep them local,
with no upload, publication, redistribution or training without separate
explicit authorization and a rights/privacy review.

## Use rules

- Public images are real-domain reference; keep them separate from simulator
  renders, and keep each ladder/session group in one split only.
- No source here has defect-visibility labels, so none supports PASS/FAIL
  training.
- Record license, URL, access date and creator per used item; store no raw data
  or weights in the repo.
