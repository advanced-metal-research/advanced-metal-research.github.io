# advanced-metal-research.github.io

Source for https://advanced-metal-research.github.io/, the landing page of the
[Advanced Metal Research](https://advancedmetalresearch.com/) GitHub organization.
It lists every public repository in the org and the write-ups we publish on GitHub Pages.

## How it works

- `build.py` (Python 3, standard library only) reads the org's **public** repositories
  from the GitHub REST API, fills `src/index.html` and writes the site to `_site/`,
  along with a `repos.json` copy of the data. A repo with no description shows the opening
  paragraph of its README instead.
- `writeups.json` is the curated list of write-up pages. Only URLs and labels go there;
  each page's title, description and preview image are read from the live page at build time.
  The repository behind a write-up site may be private, so never put a private repo's
  name or details in this file.
- `data/snapshot.json` is a committed copy of the last live data. If the API or a write-up
  page can't be reached, the build falls back to it, so a deploy never fails for lack of data.
- `.github/workflows/pages.yml` builds and deploys on every push to `main`, on manual
  dispatch and daily at 06:23 UTC.

Private repositories never appear on the page: the API query asks for `type=public`, and
`build.py` drops anything that isn't `private: false` and refuses to render a repo whose
visibility isn't `public`.

## Local build

```sh
python3 build.py                                    # live data, snapshot fallback
python3 build.py --offline                          # snapshot only
GH_TOKEN=$(gh auth token) python3 build.py --update-snapshot   # refresh the snapshot
python3 -m http.server -d _site 8000
```

## Adding a write-up

Add the post URL (under `https://advanced-metal-research.github.io/`) to `posts` in
`writeups.json`, newest first, with an optional `date` and `paper` label. Then run
`--update-snapshot` and commit. Posts link back to this page; there is no separate index
page per repository.

Design: tokens and components from the AMR website (Saira from Google Fonts, ice and ink
palette, brand red #b8020d, light and dark themes sharing the `amr-theme` preference).
