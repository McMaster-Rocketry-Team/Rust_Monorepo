import { test } from 'vitest'
import { Resampler } from './Resampler'

test('Resample 10hz to 3hz', () => {
  const data = [0 + 1, 1 + 1, 0 + 1, 1 + 1, 0 + 1, 1, 0, 1, 0, 1]
  const resampler = new Resampler(0, 10, 3, 0)
  for (const element of data) {
    const resampled = resampler.next(element)
    console.log(resampled)
  }
})
