import { defineCollection } from "astro:content";
import { glob } from "astro/loaders";
import { routeOf } from "./plugins/remark-terra.mjs";

// Every Markdown file in docs/ is a page, at the same path MkDocs gave it.
const docs = defineCollection({
    loader: glob({
        base: ".",
        pattern: ["**/*.md", "!index.md", "!node_modules/**", "!dist/**", "!src/**", "!public/**"],
        generateId: ({ entry }) => routeOf(entry),
    }),
});

export const collections = { docs };
