import React, { useMemo } from 'react'

export default function FilterBar({ waypoints, filters, onFiltersChange }) {
  const sources = useMemo(
    () => [...new Set(waypoints.map(w => w.source).filter(Boolean))].sort(),
    [waypoints]
  )

  const isSourceVisible = (s) => filters.sources === null || filters.sources.includes(s)

  const toggleSource = (s) => {
    onFiltersChange(prev => {
      const current = prev.sources === null ? sources : [...prev.sources]
      const next = current.includes(s) ? current.filter(v => v !== s) : [...current, s]
      return { ...prev, sources: next.length === sources.length ? null : next }
    })
  }

  return (
    <div style={{
      background: '#0f3460', padding: '6px 16px',
      display: 'flex', gap: '16px', alignItems: 'center',
      color: '#ccc', fontFamily: 'monospace', fontSize: '12px',
      flexShrink: 0,
    }}>
      <span style={{ color: '#a0c4ff', fontWeight: 'bold' }}>Source:</span>
      {sources.map(s => (
        <label key={s} style={{
          cursor: 'pointer', display: 'inline-flex', alignItems: 'center', gap: '5px',
          opacity: isSourceVisible(s) ? 1 : 0.35,
        }}>
          <input type="checkbox" checked={isSourceVisible(s)}
            onChange={() => toggleSource(s)} style={{ cursor: 'pointer' }} />
          {s}
        </label>
      ))}

      <span style={{ marginLeft: 'auto', color: '#555', fontSize: '11px' }}>
        {waypoints.filter(w => 'x' in w).length} waypoints · click spheres to select
      </span>
    </div>
  )
}
