'use strict';

const { test } = require('node:test');
const assert = require('node:assert/strict');
const { readFileSync, existsSync } = require('node:fs');
const path = require('node:path');

const ROOT = path.resolve(__dirname, '..');
const ARTICLE = path.join(ROOT, 'docs', 'abot-project-retro.wechat.md');

function parseFrontmatter(md) {
  const m = md.match(/^---\s*\r?\n([\s\S]*?)\r?\n---\s*\r?\n/);
  if (!m) {
    throw new Error('article is missing a YAML frontmatter block');
  }
  const meta = {};
  for (const line of m[1].split(/\r?\n/)) {
    const kv = line.match(/^([A-Za-z_][A-Za-z0-9_-]*):\s*(.*)$/);
    if (kv) {
      let value = kv[2].trim();
      if (
        (value.startsWith('"') && value.endsWith('"')) ||
        (value.startsWith("'") && value.endsWith("'"))
      ) {
        value = value.slice(1, -1);
      }
      meta[kv[1]] = value;
    }
  }
  return meta;
}

test('abot-wechat-article: WeChat article markdown with required frontmatter', () => {
  // GIVEN the aBot project retro HTML has been generated
  // WHEN the wechat publisher prepares the WeChat draft
  // THEN a WeChat-ready markdown exists with title/author/digest frontmatter and project retro body
  const md = readFileSync(ARTICLE, 'utf8');
  const meta = parseFrontmatter(md);

  assert.ok(meta.title, 'frontmatter should declare a title');
  assert.ok(meta.author, 'frontmatter should declare an author');
  assert.ok(meta.digest, 'frontmatter should declare a digest');
  assert.match(meta.title, /aBot/, 'title should mention aBot');
  assert.match(md, /数字孪生/, 'article body should discuss digital twin');
  assert.match(md, /AI 辅助开发/, 'article body should discuss AI-assisted development');
});

test('abot-wechat-article: source_url points to GitHub repo', () => {
  // GIVEN the article is configured for WeChat publishing
  // WHEN the frontmatter is parsed
  // THEN source_url maps to the GitHub repo so readers can click "阅读原文"
  const md = readFileSync(ARTICLE, 'utf8');
  const meta = parseFrontmatter(md);

  assert.equal(meta.source_url, 'https://github.com/derekhu0002/aBot', 'source_url should point to the GitHub repo');
});

test('abot-wechat-article: banner image exists', () => {
  // GIVEN the article declares a banner_path in frontmatter
  // WHEN the publisher prepares the title image
  // THEN the banner file exists on disk relative to the article directory
  const md = readFileSync(ARTICLE, 'utf8');
  const meta = parseFrontmatter(md);

  assert.ok(meta.banner_path, 'frontmatter should declare a banner_path');
  const articleDir = path.dirname(ARTICLE);
  const bannerFile = path.resolve(articleDir, meta.banner_path);
  assert.ok(existsSync(bannerFile), `banner file should exist: ${bannerFile}`);
});

test('abot-wechat-article: all referenced images exist', () => {
  // GIVEN the article references multiple images inline
  // WHEN the publisher prepares the draft
  // THEN every referenced image file exists on disk
  const md = readFileSync(ARTICLE, 'utf8');
  const articleDir = path.dirname(ARTICLE);
  const imgRefs = [...md.matchAll(/!\[.*?\]\((images\/[^)]+)\)/g)].map(m => m[1]);

  assert.ok(imgRefs.length >= 6, `article should reference at least 6 images, found ${imgRefs.length}`);
  for (const img of imgRefs) {
    const imgPath = path.resolve(articleDir, img);
    assert.ok(existsSync(imgPath), `image file should exist: ${imgPath}`);
  }
});
