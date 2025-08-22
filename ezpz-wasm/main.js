import init, { hello, benchmark, test_faer } from "./pkg/ezpz_wasm.js";
init().then(() => {
  console.log("Hello! Code is running.");
  const messageDisplay = document.getElementById("message");
  console.log("Calling test_faer");
  messageDisplay.textContent=test_faer();

  console.log("Calling benchmark");
  const startTime = performance.now()
  const runs=100;
  for (let i = 0; i < runs; i++) {
    benchmark();
  }
  const endTime = performance.now()
  console.log(`Call to 'benchmark' took ${(endTime - startTime)/runs} milliseconds each (ran ${runs} times)`)
});
