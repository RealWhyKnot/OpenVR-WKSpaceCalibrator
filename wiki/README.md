# Wiki source

The `.md` files in this folder are the **source of truth** for the GitHub Wiki at <https://github.com/RealWhyKnot/OpenVR-WKSpaceCalibrator/wiki>. They are mirrored to the wiki repo by [`.github/workflows/wiki-sync.yml`](../.github/workflows/wiki-sync.yml) on every push to `main` that touches `wiki/**`.

## Why source-control the wiki?

- Wiki edits go through normal PR review.
- Wiki content has the same git history as the code it documents.
- Forks and mirrors get a complete copy of the docs without scraping.

## Editing rules

- Always edit here, never on the GitHub web UI for the wiki. Web edits will be **overwritten** by the next sync.
- Page filename = page name. Spaces become hyphens (`Continuous-Calibration.md` ⇒ "Continuous Calibration" page).
- Wiki-style links work: `[[Page Name]]` and `[[Page Name|alt text]]`.
- `_Sidebar.md` is the sidebar (special filename, recognised by GitHub).
- This `README.md` is **excluded** from the sync -- GitHub Wiki uses `Home.md` as the landing page.

## One-time bootstrap

The wiki git repo only materialises after a maintainer creates the first page through the GitHub web UI. Before that exists, the sync workflow exits with a warning. Open the Wiki tab on the repo, click "Create the first page", save anything (it'll be overwritten on next sync), and the workflow will start succeeding.

## Pages

- [Home](Home.md) -- landing
- [Architecture](Architecture.md)
- [Continuous Calibration](Continuous-Calibration.md)
- [Driver Protocol](Driver-Protocol.md)
- [Prediction Suppression](Prediction-Suppression.md)
- [Building](Building.md)
- [Troubleshooting](Troubleshooting.md)
