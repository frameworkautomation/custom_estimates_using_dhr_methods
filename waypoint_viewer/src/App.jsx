import React, { useState, useEffect, useCallback } from 'react'
import { getWaypoints, getEdges } from './api'
import Scene from './Scene'

export default function App() {
  const [waypoints, setWaypoints] = useState([])
  const [edges, setEdges] = useState([])
  const [selected, setSelected] = useState([])

  const reload = useCallback(async () => {
    const [wps, eds] = await Promise.all([getWaypoints(), getEdges()])
    setWaypoints(wps)
    setEdges(eds)
  }, [])

  useEffect(() => { reload() }, [reload])

  const visibleWaypoints = waypoints.filter(wp => 'x' in wp)

  const handleSelect = useCallback((name) => {
    setSelected(prev =>
      prev.includes(name) ? prev.filter(n => n !== name)
      : prev.length >= 2 ? [name]
      : [...prev, name]
    )
  }, [])

  return (
    <div style={{ width: '100vw', height: '100vh' }}>
      <Scene
        waypoints={visibleWaypoints}
        edges={edges}
        selected={selected}
        onSelect={handleSelect}
      />
    </div>
  )
}
