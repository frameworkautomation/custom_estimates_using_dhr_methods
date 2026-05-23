import React, { useMemo } from 'react'

export default function FilterBar({ waypoints, filters, onFiltersChange }) {
  const sources = useMemo(
    () => [...new Set(waypoints.map(w => w.source).filter(Boolean))].sort(),
    [waypoints]
  )
  const moveTypes = useMemo(
    () => [...new Set(waypoints.map(w => w.move_type).filter(Boolean))].sort(),
    [waypoints]
  )

  // null = show all; array = whitelist of visible values
  const isVisible = (key, value) =>
    filters[key] === null || filters[key].includes(value)

  const toggle = (key, value, allValues) => {
    onFiltersChange(prev => {
      const current = prev[key] === null ? allValues : [...prev[key]]
      const next = current.includes(value)
        ? current.filter(v => v !== value)
        : [...current, value]
      // If everything checked, collapse back to null (no filter)
      return { ...prev, [key]: next.length === allValues.length ? null : next }
    })
  }

  const checkboxLabel = (checked, text) => (
    <label key={text} style={{
      cursor: 'pointer', marginLeft: '10px',
      opacity: checked ? 1 : 0.35,
      display: 'inline-flex', alignItems: 'center', gap: '4px',
    }}>
      <input type="checkbox" checked={checked} readOnly style={{ cursor: 'pointer' }} />
      {text}
    </label>
  )

  return (
    <div style={{
      background: '#0f3460', padding: '6px 16px',
      display: 'flex', gap: '8px', alignItems: 'center',
      color: '#ccc', fontFamily: 'monospace', fontSize: '12px',
      flexShrink: 0, flexWrap: 'wrap',
    }}>
      <span style={{ color: '#a0c4ff', fontWeight: 'bold' }}>Source:</span>
      {sources.map(s => (
        <span key={s} onClick={() => toggle('sources', s, sources)} style={{ cursor: 'pointer' }}>
          {checkboxLabel(isVisible('sources', s), s)}
        </span>
      ))}

      <span style={{
        borderLeft: '1px solid #333', marginLeft: '8px',
        paddingLeft: '16px', color: '#a0c4ff', fontWeight: 'bold',
      }}>
        Move type:
      </span>
      {moveTypes.map(m => (
        <span key={m} onClick={() => toggle('moveTypes', m, moveTypes)} style={{ cursor: 'pointer' }}>
          {checkboxLabel(isVisible('moveTypes', m), m)}
        </span>
      ))}

      <span style={{ marginLeft: 'auto', color: '#555', fontSize: '11px' }}>
        {waypoints.filter(w => 'x' in w).length} waypoints · click spheres to select
      </span>
    </div>
  )
}
