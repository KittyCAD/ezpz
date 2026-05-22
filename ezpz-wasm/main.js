import init, { solveText } from "./pkg/ezpz_wasm.js";

const source = `
point p
point q
p.x = 0
p.y = 0
distance(p, q, 4)

p roughly (0, 0)
q roughly (3, 2)
`;

init().then(() => {
  const output = document.getElementById("output");
  const result = solveText(source);
  output.textContent = JSON.stringify(result, null, 2);
});
