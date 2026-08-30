import { useEffect, useState } from 'react';

/** Reactive CSS media query, so layout logic and styling agree on one breakpoint. */
export function useMediaQuery(query: string): boolean {
  const [matches, setMatches] = useState(() =>
    typeof window === 'undefined' ? false : window.matchMedia(query).matches,
  );

  useEffect(() => {
    const list = window.matchMedia(query);
    const update = () => setMatches(list.matches);
    update();
    list.addEventListener('change', update);
    return () => list.removeEventListener('change', update);
  }, [query]);

  return matches;
}

/** The one breakpoint in the app: below this the panel is a sheet, above it a sidebar. */
export const DESKTOP_QUERY = '(min-width: 900px)';
