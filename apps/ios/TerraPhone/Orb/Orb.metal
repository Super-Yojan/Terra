#include <metal_stdlib>
#include <SwiftUI/SwiftUI_Metal.h>
using namespace metal;

// The Terra orb: a thin glowing ring, purple on one side fading to
// blue-white on the other, with a soft halo inside and out, on a transparent
// background. The outline wobbles slightly and the colour drifts around the
// ring over time.
//
// Analytic per pixel (no loops): a few trig calls, so it can run full-screen
// on the mounted phone without competing with ARKit for the GPU.
//
// Every time coefficient below (0.7, 1.1, 0.25) times 100 is an integer, so
// callers can wrap `time` by 2π·100 seamlessly.
//
// position: pixel in the view (points, origin top-left); size: view size.
[[ stitchable ]] half4 terraOrb(float2 position, half4 color, float2 size, float time) {
    float scale = 0.5 * min(size.x, size.y);
    float2 uv = (position - 0.5 * size) / scale;         // shorter side spans -1...1
    uv.y = -uv.y;
    float r = length(uv);
    float a = atan2(uv.y, uv.x);                          // 0 = right, π/2 = top
    float t = time;

    // Slightly irregular outline that breathes.
    float radius = 0.78 * (1.0 + 0.022 * sin(2.0 * a + 0.7 * t) + 0.014 * sin(3.0 * a - 1.1 * t));
    float d = r - radius;                                 // < 0 inside the ring

    // A soft band: crisp on the outside, fading inward, plus a faint halo.
    float band = d > 0.0 ? exp(-pow(d / 0.022, 2.0)) : exp(-pow(d / 0.06, 2.0));
    float halo = d > 0.0 ? exp(-d / 0.06) * 0.18 : 0.0;   // outside only
    float glow = band + halo;

    // Colour by angle: blue-white on the bright side, purple opposite. At the
    // home screen's still frame (t = 3) the bright side faces right; it then
    // drifts slowly around the ring.
    float side = 0.5 + 0.5 * cos(a - 0.25 * t + 0.75);    // 1 = bright side
    float3 purple = float3(0.45, 0.26, 0.92);
    float3 white = float3(0.80, 0.94, 1.0);
    float3 tint = mix(purple, white, smoothstep(0.35, 1.0, side));
    float intensity = glow * (0.7 + 0.45 * side);

    float3 rgb = clamp(tint * intensity, 0.0, 1.0);
    // Transparent where there is no glow. SwiftUI expects premultiplied
    // colour: rgb <= alpha holds because alpha is the brightest channel.
    float alpha = max(rgb.r, max(rgb.g, rgb.b));
    return half4(half3(rgb), half(alpha));
}
