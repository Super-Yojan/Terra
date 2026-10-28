// @ts-check
import { defineConfig } from "astro/config";
import sitemap from "@astrojs/sitemap";
import pitchTheme from "./src/styles/shiki-pitch.json" with { type: "json" };
import { remarkCallout } from "./src/plugins/remark-callout.mjs";
import { remarkTerra } from "./src/plugins/remark-terra.mjs";

const base = "/Terra";

// A GitHub project Page under super-yojan.dev, styled like the user site.
// The pages are the Markdown files in this folder, so they still read on GitHub.
export default defineConfig({
    site: "https://super-yojan.dev",
    base,
    integrations: [sitemap()],
    markdown: {
        remarkPlugins: [[remarkTerra, { base }], remarkCallout],
        // The blog's pitch theme: greys plus the teal ramp, one row per line.
        shikiConfig: {
            theme: pitchTheme,
            wrap: false,
        },
    },
});
