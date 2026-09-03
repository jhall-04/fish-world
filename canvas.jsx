import { useEffect, useRef, useState } from "react";

// Where boids.js / boids.wasm live, relative to the site root.
const ASSET_DIR = "/boids/";

export default function BoidsCanvas({ className }) {
  const canvasRef = useRef(null);
  const wrapperRef = useRef(null);
  const moduleRef = useRef(null);
  const startedRef = useRef(false);
  const [visible, setVisible] = useState(false);
  const [status, setStatus] = useState("idle");

  // Don't pay for a wasm download until the sim is actually on screen.
  useEffect(() => {
    const el = wrapperRef.current;
    if (!el) return;
    const io = new IntersectionObserver(
      ([entry]) => {
        if (entry.isIntersecting) {
          setVisible(true);
          io.disconnect();
        }
      },
      { rootMargin: "200px" }
    );
    io.observe(el);
    return () => io.disconnect();
  }, []);

  useEffect(() => {
    if (!visible) return;
    // StrictMode runs effects twice in dev; a second init would spawn a
    // second main loop against the same canvas.
    if (startedRef.current) return;
    startedRef.current = true;

    let cancelled = false;
    setStatus("loading");

    const script = document.createElement("script");
    script.src = ASSET_DIR + "boids.js";
    script.async = true;

    script.onload = async () => {
      if (cancelled) return;
      try {
        const instance = await window.createBoids({
          canvas: canvasRef.current,
          locateFile: (path) => ASSET_DIR + path,
          // Emscripten writes raylib's banner to stdout; keep it off the console.
          print: () => {},
          printErr: (text) => console.warn("[boids]", text),
        });
        if (cancelled) return;
        moduleRef.current = instance;
        setStatus("running");
      } catch (err) {
        console.error(err);
        setStatus("error");
      }
    };
    script.onerror = () => setStatus("error");

    document.body.appendChild(script);

    return () => {
      cancelled = true;
      const mod = moduleRef.current;
      if (mod && mod._StopSim) mod._StopSim();
      moduleRef.current = null;
      script.remove();
      delete window.createBoids;
    };
  }, [visible]);

  return (
    <div ref={wrapperRef} className={className}>
      {/* id="canvas" is required: raylib's web backend targets the
          "#canvas" selector directly for sizing and input events. */}
      <canvas
        ref={canvasRef}
        id="canvas"
        width={1000}
        height={800}
        tabIndex={-1}
        onContextMenu={(e) => e.preventDefault()}
        style={{
          display: "block",
          width: "100%",
          height: "auto",
          aspectRatio: "1000 / 800",
          touchAction: "none",
          outline: "none",
        }}
      />
      {status === "error" && <p>The simulation didn't load. Try reloading.</p>}
    </div>
  );
}