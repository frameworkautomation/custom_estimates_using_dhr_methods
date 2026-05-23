import React from 'react'
import { Line } from '@react-three/drei'

// tested: null → grey, true → green, false → red
const COLORS = { true: '#44bb44', false: '#ff4444', null: '#555577' }

function toScene(wp) {
  return [wp.x ?? 0, wp.z ?? 0, wp.y ?? 0]
}

export default function Edges({ edges, waypointMap }) {
  return (
    <>
      {edges.map((edge, i) => {
        const from = waypointMap[edge.from]
        const to = waypointMap[edge.to]
        // Skip edges whose waypoints aren't in the current filtered view
        if (!from || !to || !('x' in from) || !('x' in to)) return null

        const testedKey = edge.tested === null ? 'null' : String(edge.tested)
        const color = COLORS[testedKey] ?? '#555577'

        return (
          <Line
            key={`${edge.from}__${edge.to}__${i}`}
            points={[toScene(from), toScene(to)]}
            color={color}
            lineWidth={1}
          />
        )
      })}
    </>
  )
}
