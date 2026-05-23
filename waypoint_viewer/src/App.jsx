import React, { useState, useEffect, useCallback } from 'react'
import { getWaypoints, getEdges, createEdge, deleteEdge } from './api'
import Scene from './Scene'
import InfoPanel from './InfoPanel'
import FilterBar from './FilterBar'

export default function App() {
  const [waypoints, setWaypoints] = useState([])
  const [edges, setEdges] = useState([])
  const [selected, setSelected] = useState([])   // array of name strings, max 2
  const [filters, setFilters] = useState({ sources: null, moveTypes: null })

  const reload = useCallback(async () => {
    const [wps, eds] = await Promise.all([getWaypoints(), getEdges()])
    setWaypoints(wps)
    setEdges(eds)
  }, [])

  useEffect(() => { reload() }, [reload])

  const visibleWaypoints = waypoints.filter(wp => {
    if (!('x' in wp)) return false
    if (filters.sources && !filters.sources.includes(wp.source)) return false
    if (filters.moveTypes && !filters.moveTypes.includes(wp.move_type)) return false
    return true
  })

  const handleSelect = useCallback((name) => {
    setSelected(prev =>
      prev.includes(name) ? prev.filter(n => n !== name)
      : prev.length >= 2 ? [name]
      : [...prev, name]
    )
  }, [])

  const handleCreateEdge = useCallback(async () => {
    if (selected.length !== 2) return
    await createEdge(selected[0], selected[1])
    setSelected([])
    await reload()
  }, [selected, reload])

  const handleDeleteEdge = useCallback(async (fromName, toName) => {
    await deleteEdge(fromName, toName)
    await reload()
  }, [reload])

  const selectedWaypoints = selected
    .map(name => waypoints.find(w => w.name === name))
    .filter(Boolean)

  return (
    <div style={{ display: 'flex', flexDirection: 'column', height: '100vh' }}>
      <FilterBar waypoints={waypoints} filters={filters} onFiltersChange={setFilters} />
      <div style={{ display: 'flex', flex: 1, overflow: 'hidden' }}>
        <div style={{ flex: 1, position: 'relative' }}>
          <Scene
            waypoints={visibleWaypoints}
            edges={edges}
            selected={selected}
            onSelect={handleSelect}
          />
        </div>
        <InfoPanel
          selected={selectedWaypoints}
          edges={edges}
          onCreateEdge={handleCreateEdge}
          onDeleteEdge={handleDeleteEdge}
        />
      </div>
    </div>
  )
}
