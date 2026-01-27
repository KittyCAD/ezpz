Aside from the tests added here, there's also a test in the trim branch that reflects this too

This PR addresses that by changing how the “point is within the arc’s angular span” part of the constraint is expressed
• It still enforces “point is on the circle” (distance-to-center equals radius)
• The angle-range residuals use the arc’s own orientation (start → end) for a consistent CCW/CW sign
• The angle-range residuals remain one-sided penalties (0 when satisfied, positive when violated)
• The angular penalties are disabled when the point is already sufficiently close to the circle, preventing the solver from pulling a near-satisfied point around

That combination is meant to stop the solver from getting a big, misleading “gradient signal” from the angular terms when it shouldn’t
