import { Butterworth2ndLP, CascadedFilter, Filter, NoopFilter } from "./Filter"

export class Resampler {
  private filter: Filter
  private sourceSampleDuration: number
  private targetSampleDuration: number
  private sourceI = 0
  private sampleI = 0
  private nextSampleTimestamp: number
  private lastReading: number | undefined

  constructor(
    private sourceTimestampStart: number,
    sourceSampleRate: number,
    targetSampleRate: number,
    private targetSampleOffset: number,
  ) {
    console.log(
      'sourceSampleRate',
      sourceSampleRate,
      'targetSampleRate',
      targetSampleRate,
    )

    if (sourceSampleRate > targetSampleRate) {
      this.filter = new CascadedFilter([
        new Butterworth2ndLP(sourceSampleRate, targetSampleRate / 4),
        new Butterworth2ndLP(sourceSampleRate, targetSampleRate / 4),
      ])
    } else {
      this.filter = new NoopFilter()
    }

    this.sourceSampleDuration = 1000 / sourceSampleRate
    this.targetSampleDuration = 1000 / targetSampleRate

    this.nextSampleTimestamp = this.targetSampleOffset
    if (this.nextSampleTimestamp < 0) {
      this.sampleI++
      this.nextSampleTimestamp += this.targetSampleDuration
    }
  }

  next(reading: number): Array<{
    timestamp: number
    reading: number
  }> {
    let filteredReading = this.filter.process(reading)
    if (this.lastReading === undefined) {
      // let the filter reach steady state
      while (Math.abs(filteredReading - reading) / reading > 0.01) {
        filteredReading = this.filter.process(reading)
      }

      this.lastReading = filteredReading
      return []
    }

    const interpolatableStart = (this.sourceI - 1) * this.sourceSampleDuration
    const interpolatableEnd = interpolatableStart + this.sourceSampleDuration

    this.sourceI++

    const results = []
    while (
      this.nextSampleTimestamp >= interpolatableStart &&
      this.nextSampleTimestamp <= interpolatableEnd
    ) {
      const t =
        (this.nextSampleTimestamp - interpolatableStart) /
        this.sourceSampleDuration
      results.push({
        timestamp: this.sourceTimestampStart + this.nextSampleTimestamp,
        reading: this.lerp(this.lastReading, filteredReading, t),
      })

      this.sampleI++
      this.nextSampleTimestamp =
        this.sampleI * this.targetSampleDuration + this.targetSampleOffset
    }

    this.lastReading = filteredReading
    return results
  }

  private lerp(a: number, b: number, t: number) {
    return a + t * (b - a)
  }
}

