import React from "react";
import ReactDOM from "react-dom/client";
import App from "./App";

type ErrorBoundaryState = { error: Error | null };

class ErrorBoundary extends React.Component<React.PropsWithChildren, ErrorBoundaryState> {
  state: ErrorBoundaryState = { error: null };

  static getDerivedStateFromError(error: Error) {
    return { error };
  }

  componentDidCatch(error: Error, info: React.ErrorInfo) {
    console.error("React render error", error, info);
  }

  render() {
    if (!this.state.error) return this.props.children;
    return (
      <main style={{ padding: 24, fontFamily: "system-ui, sans-serif", color: "#991b1b" }}>
        <h1>界面渲染错误</h1>
        <pre style={{ whiteSpace: "pre-wrap" }}>{this.state.error.stack || this.state.error.message}</pre>
      </main>
    );
  }
}

window.addEventListener("error", (event) => {
  console.error("Window error", event.error || event.message);
});

window.addEventListener("unhandledrejection", (event) => {
  console.error("Unhandled rejection", event.reason);
});

ReactDOM.createRoot(document.getElementById("root") as HTMLElement).render(
  <React.StrictMode>
    <ErrorBoundary>
      <App />
    </ErrorBoundary>
  </React.StrictMode>,
);
