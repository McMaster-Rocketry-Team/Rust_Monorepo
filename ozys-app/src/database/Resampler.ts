import { Biquad, biquadLP } from '@thi.ng/dsp'
import { RingBuffer } from 'ring-buffer-ts'

export class Resampler {
  private filter: Biquad
  private cubicBuffer: RingBuffer<number> = new RingBuffer(4)
  private sourceSampleDuration: number
  private targetSampleDuration: number
  private sourceI = 0
  private sampleI = 0
  private nextSampleTimestamp: number

  constructor(
    sourceSampleRate: number,
    targetSampleRate: number,
    private targetSampleOffset: number,
  ) {
    // TODO handle case when targetSampleRate > sourceSampleRate
    this.filter = biquadLP(targetSampleRate / sourceSampleRate)

    this.sourceSampleDuration = 1000 / sourceSampleRate
    this.targetSampleDuration = 1000 / targetSampleRate
    console.log('sourceSampleDuration', this.sourceSampleDuration)
    console.log('targetSampleDuration', this.targetSampleDuration)

    this.nextSampleTimestamp = this.targetSampleOffset
    if (this.nextSampleTimestamp < 0) {
      this.nextSampleTimestamp += this.targetSampleDuration
    }
  }

  next(reading: number): {
    timestamp: number
    reading: number
  } | null {
    let filteredReading = this.filter.next(reading)
    if (this.cubicBuffer.isEmpty()) {
      // let the filter reach steady state
      while (Math.abs(filteredReading - reading) / reading > 0.01) {
        filteredReading = this.filter.next(reading)
      }

      for (let i = 0; i < 4; i++) {
        this.cubicBuffer.add(filteredReading)
      }
    } else {
      this.cubicBuffer.add(filteredReading)
    }

    const interpolatableStart = (this.sourceI - 2) * this.sourceSampleDuration
    const interpolatableEnd = interpolatableStart + this.sourceSampleDuration

    this.sourceI++

    if (
      this.nextSampleTimestamp >= interpolatableStart &&
      this.nextSampleTimestamp <= interpolatableEnd
    ) {
      const t =
        (this.nextSampleTimestamp - interpolatableStart) /
        this.sourceSampleDuration
      const result = {
        timestamp: this.nextSampleTimestamp,
        reading: this.cubicInterpolate(t),
      }

      this.sampleI++
      this.nextSampleTimestamp =
        this.sampleI * this.targetSampleDuration + this.targetSampleOffset
      return result
    }

    return null
  }

  // interpolate using a Catmull-Rom Spline
  private cubicInterpolate(t: number) {
    const p0 = this.cubicBuffer.get(0)!
    const p1 = this.cubicBuffer.get(1)!
    const p2 = this.cubicBuffer.get(2)!
    const p3 = this.cubicBuffer.get(3)!
    // console.log(p0, p1, p2, p3)
    return (
      0.5 *
      (2 * p1 +
        (-p0 + p2) * t +
        (2 * p0 - 5 * p1 + 4 * p2 - p3) * t * t +
        (-p0 + 3 * p1 - 3 * p2 + p3) * t * t * t)
    )
  }
}
