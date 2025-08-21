import init, { hello } from "./pkg/ezpz_wasm.js";
init().then(() => {
  console.log("Hello! Code is running.");
  const messageDisplay = document.getElementById("message");
  messageDisplay.textContent=hello();
});
