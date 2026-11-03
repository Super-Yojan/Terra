import Foundation
import simd

struct RotationOffsetSample {
    let timestamp: Double
    let cameraPositionWorld: SIMD2<Double>
    let roverHeadingWorld: Double
}

struct RotationOffsetFit {
    let forward: Double
    let left: Double
    let rmsError: Double
    let headingSpan: Double
    let sampleCount: Int
    let turnDirection: Int
}

enum RotationOffsetFitter {
    static func fit(_ samples: [RotationOffsetSample]) -> RotationOffsetFit? {
        guard samples.count >= 30,
              let first = samples.first,
              let last = samples.last,
              samples.allSatisfy({ $0.timestamp.isFinite && $0.cameraPositionWorld.x.isFinite && $0.cameraPositionWorld.y.isFinite && $0.roverHeadingWorld.isFinite }),
              zip(samples, samples.dropFirst()).allSatisfy({ pair in pair.0.timestamp < pair.1.timestamp }),
              last.timestamp - first.timestamp >= 2 else { return nil }

        var angles = [Double](repeating: 0, count: samples.count)
        var accumulated = 0.0
        for index in 1..<samples.count {
            let delta = samples[index].roverHeadingWorld - samples[index - 1].roverHeadingWorld
            accumulated += atan2(sin(delta), cos(delta))
            angles[index] = accumulated
        }
        let span = abs(accumulated)
        guard span >= .pi / 2 else { return nil }

        // Fit dPosition = (R(theta) - I) * offset in the rover frame at start.
        var aa = 0.0, ab = 0.0, bb = 0.0, ax = 0.0, bx = 0.0
        let c0 = cos(first.roverHeadingWorld), s0 = sin(first.roverHeadingWorld)
        for index in 1..<samples.count {
            let c = cos(angles[index]), s = sin(angles[index])
            let dxWorld = samples[index].cameraPositionWorld - first.cameraPositionWorld
            let dx = c0 * dxWorld.x + s0 * dxWorld.y
            let dy = -s0 * dxWorld.x + c0 * dxWorld.y
            accumulate(a: c - 1, b: -s, value: dx, aa: &aa, ab: &ab, bb: &bb, ax: &ax, bx: &bx)
            accumulate(a: s, b: c - 1, value: dy, aa: &aa, ab: &ab, bb: &bb, ax: &ax, bx: &bx)
        }
        let determinant = aa * bb - ab * ab
        guard determinant.isFinite, determinant > 1e-6 else { return nil }
        let forward = (ax * bb - bx * ab) / determinant
        let left = (bx * aa - ax * ab) / determinant
        guard forward.isFinite, left.isFinite, hypot(forward, left) <= 2 else { return nil }

        // Estimate the fixed pivot from all samples and score the trajectory residual.
        var centre = SIMD2<Double>.zero
        for sample in samples {
            let c = cos(sample.roverHeadingWorld), s = sin(sample.roverHeadingWorld)
            centre += sample.cameraPositionWorld - SIMD2(c * forward - s * left, s * forward + c * left)
        }
        centre /= Double(samples.count)
        var squaredError = 0.0
        for sample in samples {
            let c = cos(sample.roverHeadingWorld), s = sin(sample.roverHeadingWorld)
            let predicted = centre + SIMD2(c * forward - s * left, s * forward + c * left)
            let error = simd_distance(sample.cameraPositionWorld, predicted)
            squaredError += error * error
        }
        let rms = sqrt(squaredError / Double(samples.count))
        guard rms.isFinite, rms <= 0.05 else { return nil }
        return RotationOffsetFit(forward: forward, left: left, rmsError: rms,
                                 headingSpan: span, sampleCount: samples.count,
                                 turnDirection: accumulated >= 0 ? 1 : -1)
    }

    private static func accumulate(a: Double, b: Double, value: Double,
                                   aa: inout Double, ab: inout Double, bb: inout Double,
                                   ax: inout Double, bx: inout Double) {
        aa += a * a; ab += a * b; bb += b * b
        ax += a * value; bx += b * value
    }
}
