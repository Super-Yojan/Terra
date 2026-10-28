function textOf(node) {
    if (!node) return "";
    if (typeof node.value === "string") return node.value;
    if (!node.children) return "";
    return node.children.map(textOf).join("");
}

function walk(node, visit) {
    visit(node);
    if (!node.children) return;
    for (const child of node.children) walk(child, visit);
}

/** Mark warning blockquotes so the page can draw them as a notice, not a pull quote. */
export function remarkCallout() {
    return (tree) => {
        walk(tree, (node) => {
            if (node.type !== "blockquote") return;
            if (!textOf(node).includes("⚠")) return;
            node.data = node.data ?? {};
            node.data.hProperties = {
                ...(node.data.hProperties ?? {}),
                class: "callout",
                role: "note",
            };
        });
    };
}
