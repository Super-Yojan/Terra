import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

const DOCS = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "../..");
const REPO = path.resolve(DOCS, "..");
const GITHUB = "https://github.com/Super-Yojan/Terra";

/** Page route for a Markdown file under docs/: README and index name their folder. */
export function routeOf(rel) {
    return rel.replace(/\.md$/, "").replace(/(^|\/)(README|index)$/, "");
}

function textOf(node) {
    if (!node) return "";
    if (typeof node.value === "string") return node.value;
    if (!node.children) return "";
    return node.children.map(textOf).join("");
}

function walk(node, visit, parent) {
    visit(node, parent);
    if (!node.children) return;
    for (const child of [...node.children]) walk(child, visit, node);
}

const escape = (s) => s.replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;");

const LABELS = { NOTE: "Note", TIP: "Tip", IMPORTANT: "Important", WARNING: "Warning", CAUTION: "Caution" };

/**
 * The Terra pages are plain Markdown that also reads well on GitHub.
 * This turns the GitHub conventions into the site's:
 * - the first `# Title` becomes the page title,
 * - `> [!TIP]` alerts become callouts,
 * - mermaid fences are left for the browser to draw,
 * - links to other `.md` pages point at their routes, and links to
 *   anything else in the repository point at GitHub.
 */
export function remarkTerra({ base }) {
    return (tree, file) => {
        const source = file.path ? path.resolve(file.path) : null;
        const fm = (file.data.astro ??= {}).frontmatter ??= {};

        // Title and description.
        const h1 = tree.children.findIndex((n) => n.type === "heading" && n.depth === 1);
        if (h1 !== -1) {
            fm.title ??= textOf(tree.children[h1]).trim();
            tree.children.splice(h1, 1);
        }

        walk(tree, (node, parent) => {
            if (node.type === "code" && node.lang === "mermaid") {
                node.type = "html";
                node.value = `<div class="diagram"><pre class="mermaid">${escape(node.value)}</pre></div>`;
                delete node.lang;
                return;
            }

            if (node.type === "blockquote") {
                const first = node.children[0];
                const lead = first?.type === "paragraph" ? first.children[0] : null;
                const m = lead?.type === "text" && lead.value.match(/^\[!(\w+)\]\s*/);
                if (m) {
                    const kind = m[1].toUpperCase();
                    lead.value = lead.value.slice(m[0].length);
                    if (!lead.value) first.children.shift();
                    if (first.children[0]?.type === "break") first.children.shift();
                    // A bold first line, `> **TL;DR**`, names the callout.
                    let label = LABELS[kind] ?? kind;
                    if (first.children[0]?.type === "strong") {
                        label = textOf(first.children.shift()).trim();
                        const rest = first.children[0];
                        if (rest?.type === "break") first.children.shift();
                        else if (rest?.type === "text") rest.value = rest.value.replace(/^\s+/, "");
                        if (rest?.type === "text" && !rest.value) first.children.shift();
                    }
                    if (first.children.length === 0) node.children.shift();
                    node.children.unshift({
                        type: "paragraph",
                        data: { hProperties: { className: ["callout-label", "t-label"] } },
                        children: [{ type: "text", value: label }],
                    });
                    node.data = { hProperties: { className: ["callout", `callout-${kind.toLowerCase()}`], role: "note" } };
                }
                return;
            }

            if ((node.type === "link" || node.type === "definition") && source) {
                node.url = rewrite(node.url, source, base);
            }
        });

        if (!fm.description) {
            const para = tree.children.find((n) => n.type === "paragraph" && n.children?.[0]?.type !== "image");
            const text = textOf(para).replace(/\s+/g, " ").trim();
            if (text) fm.description = text.length > 180 ? `${text.slice(0, 177).replace(/\s+\S*$/, "")}…` : text;
        }
    };
}

function rewrite(url, source, base) {
    if (!url || /^([a-z][a-z0-9+.-]*:|#|\/\/)/i.test(url)) return url;
    if (url.startsWith("/")) return url;
    const [target, hash = ""] = url.split(/(?=#)/);
    if (!target) return url;
    const abs = path.resolve(path.dirname(source), decodeURI(target));
    const rel = path.relative(DOCS, abs).split(path.sep).join("/");
    if (!rel.startsWith("..") && rel.endsWith(".md")) {
        const route = routeOf(rel);
        return `${base}/${route ? `${route}/` : ""}${hash}`;
    }
    const repoRel = path.relative(REPO, abs).split(path.sep).join("/");
    if (repoRel.startsWith("..")) return url;
    const kind = fs.existsSync(abs) && fs.statSync(abs).isDirectory() ? "tree" : "blob";
    return `${GITHUB}/${kind}/main/${repoRel}${hash}`;
}
