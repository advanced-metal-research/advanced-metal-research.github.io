#!/usr/bin/env python3
"""Build the Advanced Metal Research GitHub organization page.

Lists every PUBLIC repository of the org (GitHub REST API, falling back to the
committed data/snapshot.json) and the curated write-ups in writeups.json, then
renders src/index.html into _site/. Standard library only.

  python3 build.py                     # live data, fallback to the snapshot
  python3 build.py --offline           # snapshot only, no network
  python3 build.py --update-snapshot   # also rewrite data/snapshot.json

Set GITHUB_TOKEN (or GH_TOKEN) for a higher API rate limit; public listing
works without one.
"""

from __future__ import annotations

import argparse
import datetime as dt
import html
import json
import os
import re
import shutil
import sys
import urllib.error
import urllib.parse
import urllib.request
from html.parser import HTMLParser
from pathlib import Path

ORG = "advanced-metal-research"
SITE_REPO = f"{ORG}.github.io"
SITE_URL = f"https://{ORG}.github.io/"
API = "https://api.github.com"
ROOT = Path(__file__).resolve().parent
SNAPSHOT = ROOT / "data" / "snapshot.json"
WRITEUPS = ROOT / "writeups.json"
SRC = ROOT / "src"
UA = f"{SITE_REPO} build"
TIMEOUT = 20


def log(msg: str) -> None:
    print(msg, file=sys.stderr)


def warn(msg: str) -> None:
    # Shows up as an annotation in GitHub Actions, as plain text elsewhere.
    prefix = "::warning::" if os.environ.get("GITHUB_ACTIONS") else "warning: "
    print(prefix + msg, file=sys.stderr)


# ---------------------------------------------------------------- HTTP


def _token() -> str | None:
    return os.environ.get("GITHUB_TOKEN") or os.environ.get("GH_TOKEN")


def http_get(url: str, accept: str = "application/vnd.github+json", api: bool = True):
    headers = {"User-Agent": UA, "Accept": accept}
    if api:
        headers["X-GitHub-Api-Version"] = "2022-11-28"
        if _token():
            headers["Authorization"] = f"Bearer {_token()}"
    req = urllib.request.Request(url, headers=headers)
    with urllib.request.urlopen(req, timeout=TIMEOUT) as resp:
        body = resp.read()
        charset = resp.headers.get_content_charset() or "utf-8"
        return body.decode(charset, "replace"), resp.headers


def api_json(path: str):
    body, _ = http_get(API + path)
    return json.loads(body)


def api_paginate(path: str) -> list:
    url, items = API + path, []
    while url:
        body, headers = http_get(url)
        items.extend(json.loads(body))
        m = re.search(r'<([^>]+)>;\s*rel="next"', headers.get("Link") or "")
        url = m.group(1) if m else None
    return items


# ---------------------------------------------------------------- repositories


def readme_excerpt(name: str, limit: int = 260) -> str | None:
    """First prose paragraph of a repo README, stripped of Markdown."""
    try:
        text, _ = http_get(f"{API}/repos/{ORG}/{name}/readme", accept="application/vnd.github.raw")
    except urllib.error.HTTPError:
        return None
    para: list[str] = []
    in_code = False
    for line in text.splitlines():
        s = line.strip()
        if s.startswith("```"):
            in_code = not in_code
            continue
        if in_code:
            continue
        if not s:
            if para:
                break
            continue
        if s.startswith(("#", "!", "[![", "<", "|", ">", "-", "*", "=")):
            if para:
                break
            continue
        para.append(s)
    if not para:
        return None
    out = " ".join(para)
    out = re.sub(r"!\[[^\]]*\]\([^)]*\)", "", out)
    out = re.sub(r"\[([^\]]+)\]\([^)]*\)", r"\1", out)
    out = re.sub(r"[`*_]{1,2}([^`*_]+)[`*_]{1,2}", r"\1", out)
    out = re.sub(r"\s+", " ", out).strip()
    if len(out) > limit:
        cut = out[:limit]
        end = cut.rfind(". ")
        out = cut[: end + 1] if end > limit // 2 else cut.rsplit(" ", 1)[0] + "…"
    return out


def pages_url(name: str) -> str:
    try:
        return api_json(f"/repos/{ORG}/{name}/pages")["html_url"]
    except (urllib.error.URLError, KeyError, ValueError):
        if name.lower() == SITE_REPO.lower():
            return SITE_URL
        return f"{SITE_URL}{name}/"


def fetch_repos() -> list[dict]:
    raw = api_paginate(f"/orgs/{ORG}/repos?type=public&per_page=100")
    repos = []
    for r in raw:
        # type=public already excludes private/internal repos; filter again so a
        # broader token or an API change can never leak one onto the page.
        if r.get("private") is not False or r.get("visibility", "public") != "public":
            continue
        desc = (r.get("description") or "").strip()
        source = "description"
        if not desc:
            desc = readme_excerpt(r["name"]) or ""
            source = "readme" if desc else None
        repos.append(
            {
                "name": r["name"],
                "url": r["html_url"],
                "visibility": "public",
                "description": desc,
                "description_source": source,
                "language": r.get("language"),
                "topics": sorted(r.get("topics") or []),
                "stars": r.get("stargazers_count", 0),
                "forks": r.get("forks_count", 0),
                "license": ((r.get("license") or {}).get("spdx_id") or None),
                "homepage": (r.get("homepage") or "").strip() or None,
                "pages_url": pages_url(r["name"]) if r.get("has_pages") else None,
                "archived": bool(r.get("archived")),
                "fork": bool(r.get("fork")),
                "pushed_at": r.get("pushed_at"),
                "created_at": r.get("created_at"),
            }
        )
    return repos


# ---------------------------------------------------------------- write-ups


class _Meta(HTMLParser):
    def __init__(self) -> None:
        super().__init__(convert_charrefs=True)
        self.meta: dict[str, str] = {}
        self.title = ""
        self._in_title = False

    def handle_starttag(self, tag, attrs):
        a = dict(attrs)
        if tag == "title":
            self._in_title = True
        elif tag == "meta":
            key = a.get("property") or a.get("name")
            if key and a.get("content") is not None and key not in self.meta:
                self.meta[key] = a["content"].strip()

    def handle_endtag(self, tag):
        if tag == "title":
            self._in_title = False

    def handle_data(self, data):
        if self._in_title:
            self.title += data


def page_meta(url: str) -> dict:
    body, _ = http_get(url, accept="text/html", api=False)
    p = _Meta()
    p.feed(body)
    image = p.meta.get("og:image")
    # og:title is the bare post title; <title> may carry a site suffix.
    return {
        "title": p.meta.get("og:title") or re.sub(r"\s+", " ", p.title).strip(),
        "description": p.meta.get("description") or p.meta.get("og:description"),
        "image": urllib.parse.urljoin(url, image) if image else None,
    }


def resolve_writeups(config: dict, previous: list[dict], offline: bool) -> list[dict]:
    """Fill each configured post with its live title/description/image."""
    prev = {p["url"]: p for p in previous if "url" in p}

    def resolve(entry: dict) -> dict:
        url = entry["url"]
        if not url.startswith(SITE_URL):
            raise SystemExit(f"writeups.json: {url} is not hosted under {SITE_URL}")
        out = dict(entry)
        live = {}
        if not offline:
            try:
                live = page_meta(url)
            except (urllib.error.URLError, OSError, ValueError) as e:
                warn(f"write-up {url}: {e}; using the snapshot")
        for key in ("title", "description", "image"):
            out[key] = entry.get(key) or live.get(key) or prev.get(url, {}).get(key)
        if not out.get("title"):
            out["title"] = url
        return out

    return [resolve(p) for p in config.get("posts", [])]


# ---------------------------------------------------------------- rendering

e = html.escape


def fmt_date(iso: str | None) -> str:
    if not iso:
        return ""
    d = dt.datetime.fromisoformat(iso.replace("Z", "+00:00"))
    return f"{d.day} {d:%b %Y}"


def iso_date(iso: str | None) -> str:
    return iso[:10] if iso else ""


def repo_card(r: dict, idx: int) -> str:
    is_site = r["name"].lower() == SITE_REPO.lower()
    meta = []
    if r["language"]:
        meta.append(f'<span class="lbl"><b>{e(r["language"])}</b></span>')
    if r["license"] and r["license"] != "NOASSERTION":
        meta.append(f'<span class="lbl">{e(r["license"])}</span>')
    if is_site:
        meta.append('<span class="lbl">This page</span>')
    if r["archived"]:
        meta.append('<span class="lbl">Archived</span>')
    if r["fork"]:
        meta.append('<span class="lbl">Fork</span>')

    if r["description"]:
        cls = ' class="from-readme"' if r["description_source"] == "readme" else ""
        note = '<span class="src">From the README</span> ' if r["description_source"] == "readme" else ""
        desc = f"<p{cls}>{note}{e(r['description'])}</p>"
    else:
        desc = '<p class="muted">No description yet.</p>'

    topics = ""
    if r["topics"]:
        chips = "".join(f'<li class="chip">{e(t)}</li>' for t in r["topics"])
        topics = f'<ul class="topics" aria-label="Topics">{chips}</ul>'

    links = [f'<a class="btn" href="{e(r["url"])}">Repository<span class="sr"> {e(r["name"])}</span></a>']
    seen = {r["url"].rstrip("/")}
    for label, href in (("Pages site", r["pages_url"]), ("Website", r["homepage"])):
        if href and href.rstrip("/") not in seen and not (is_site and href.rstrip("/") == SITE_URL.rstrip("/")):
            seen.add(href.rstrip("/"))
            links.append(f'<a class="link" href="{e(href)}">{label}<span class="sr"> for {e(r["name"])}</span></a>')

    star_word = "star" if r["stars"] == 1 else "stars"
    return f"""
      <li class="repo">
        <div class="repo-head"><span class="lbl num">{idx:02d}</span>{''.join(meta)}</div>
        <h3><a href="{e(r['url'])}">{e(r['name'])}</a></h3>
        {desc}
        {topics}
        <dl class="repo-stats">
          <div><dt>Stars</dt><dd class="num">{r['stars']}<span class="sr"> {star_word}</span></dd></div>
          <div><dt>Updated</dt><dd><time datetime="{iso_date(r['pushed_at'])}">{fmt_date(r['pushed_at'])}</time></dd></div>
        </dl>
        <div class="actions">{''.join(links)}</div>
      </li>"""


def writeups_html(posts: list[dict]) -> str:
    if not posts:
        return '<p class="muted">No write-ups yet.</p>'
    items = []
    n = len(posts)
    for k, p in enumerate(posts):
        i = n - k  # posts are listed newest first and numbered oldest = 01
        thumb = ""
        if p.get("image"):
            thumb = (
                f'<a class="thumb" href="{e(p["url"])}" tabindex="-1" aria-hidden="true">'
                f'<img src="{e(p["image"])}" alt="" loading="lazy" decoding="async"></a>'
            )
        meta = [f'<span class="lbl"><b>No. {i:02d}</b></span>']
        if p.get("date"):
            meta.append(f'<span class="lbl"><time datetime="{e(p["date"])}">{fmt_date(p["date"])}</time></span>')
        if p.get("paper"):
            meta.append(f'<span class="lbl">{e(p["paper"])}</span>')
        desc = f"<p>{e(p['description'])}</p>" if p.get("description") else ""
        items.append(
            f"""
      <li class="{'has-thumb' if thumb else ''}">
        {thumb}
        <div>
          <div class="meta">{''.join(meta)}</div>
          <h3><a href="{e(p['url'])}">{e(p['title'])}</a></h3>
          {desc}
        </div>
      </li>"""
        )
    return f'\n    <ol class="posts">{"".join(items)}\n    </ol>'


def archive_html(repos: list[dict]) -> str:
    if not repos:
        return ""
    rows = []
    for r in repos:
        kind = " · ".join(k for k, on in (("Archived", r["archived"]), ("Fork", r["fork"])) if on)
        desc = e(r["description"]) if r["description"] else '<span class="muted">No description.</span>'
        rows.append(
            f'<li><a class="link" href="{e(r["url"])}">{e(r["name"])}</a>'
            f'<span class="lbl">{kind}</span><span>{desc}</span></li>'
        )
    return f"""
    <details class="archive">
      <summary><span class="lbl"><b>Archived and forks</b> &nbsp; {len(repos)}</span></summary>
      <ul class="rows">{''.join(rows)}</ul>
    </details>"""


def render(data: dict, out_dir: Path) -> None:
    repos = data["repos"]
    for r in repos:
        if r.get("visibility") != "public":
            raise SystemExit("refusing to render a non-public repository")
    active = [r for r in repos if not r["archived"] and not r["fork"]]
    other = [r for r in repos if r["archived"] or r["fork"]]
    # Most recently pushed first; this site's own repo last.
    active.sort(key=lambda r: (r["name"].lower() == SITE_REPO.lower(), -(dt.datetime.fromisoformat((r["pushed_at"] or "1970-01-01T00:00:00Z").replace("Z", "+00:00")).timestamp())))
    other.sort(key=lambda r: r["name"].lower())

    langs = sorted({r["language"] for r in active if r["language"]})
    n_posts = len(data["writeups"])
    built = dt.datetime.fromisoformat(data["generated_at"].replace("Z", "+00:00"))
    values = {
        "REPO_COUNT": str(len(active)),
        "REPO_WORD": "repository" if len(active) == 1 else "repositories",
        "STAR_COUNT": str(sum(r["stars"] for r in active)),
        "LANG_COUNT": str(len(langs)),
        "LANG_LIST": e(", ".join(langs) or "None"),
        "POST_COUNT": str(n_posts),
        "BUILT_ISO": built.strftime("%Y-%m-%dT%H:%MZ"),
        "BUILT_HUMAN": f"{built.day} {built:%b %Y}, {built:%H:%M} UTC",
        "DATA_SOURCE": "GitHub API" if data.get("source") == "live" else "saved snapshot",
        "REPO_CARDS": "".join(repo_card(r, i) for i, r in enumerate(active, 1)),
        "ARCHIVE": archive_html(other),
        "WRITEUPS": writeups_html(data["writeups"]),
    }
    page = (SRC / "index.html").read_text()
    for key, val in values.items():
        page = page.replace("{{" + key + "}}", val)
    left = re.findall(r"\{\{[A-Z_]+\}\}", page)
    if left:
        raise SystemExit(f"unfilled placeholders: {sorted(set(left))}")

    if out_dir.exists():
        shutil.rmtree(out_dir)
    shutil.copytree(SRC, out_dir)
    (out_dir / "index.html").write_text(page)
    public = {k: data[k] for k in ("generated_at", "org", "repos", "writeups")}
    (out_dir / "repos.json").write_text(json.dumps(public, indent=2) + "\n")


# ---------------------------------------------------------------- main


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--offline", action="store_true", help="use data/snapshot.json only")
    ap.add_argument("--update-snapshot", action="store_true", help="rewrite data/snapshot.json from live data")
    ap.add_argument("--out", default=str(ROOT / "_site"))
    args = ap.parse_args()

    snapshot = json.loads(SNAPSHOT.read_text()) if SNAPSHOT.exists() else {"repos": [], "writeups": []}
    config = json.loads(WRITEUPS.read_text())
    now = dt.datetime.now(dt.timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")

    repos, source = None, "snapshot"
    if not args.offline:
        try:
            repos, source = fetch_repos(), "live"
            log(f"fetched {len(repos)} public repositories from the GitHub API")
        except (urllib.error.URLError, OSError, ValueError) as e_:
            warn(f"GitHub API failed ({e_}); using data/snapshot.json")
    if repos is None:
        if not snapshot["repos"]:
            raise SystemExit("no live data and no snapshot")
        repos = snapshot["repos"]

    data = {
        "generated_at": now if source == "live" else snapshot.get("generated_at", now),
        "org": ORG,
        "source": source,
        "repos": repos,
        "writeups": resolve_writeups(config, snapshot.get("writeups", []), args.offline),
    }
    render(data, Path(args.out))
    log(f"wrote {args.out}/index.html ({source}: {len(repos)} repos, "
        f"{len(data['writeups'])} write-ups)")

    if args.update_snapshot:
        if source != "live":
            raise SystemExit("--update-snapshot needs live data")
        snap = {k: data[k] for k in ("generated_at", "org", "repos", "writeups")}
        SNAPSHOT.write_text(json.dumps(snap, indent=2) + "\n")
        log(f"updated {SNAPSHOT.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
