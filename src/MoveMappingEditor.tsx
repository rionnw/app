import { invoke } from "@tauri-apps/api/core";
import { save as dialogSave } from "@tauri-apps/plugin-dialog";
import { useEffect, useMemo, useState } from "react";

export type MoveMapping = {
  mnemonics: string[];
  digits: string[];
};

const DIGITS_0_9 = ["0", "1", "2", "3", "4", "5", "6", "7", "8", "9"] as const;

function validatePermutation(digits: string[]): string | null {
  if (digits.length !== 10) return `必须有 10 项，当前 ${digits.length}`;
  const trimmed = digits.map((d) => d.trim());
  if (trimmed.some((d) => d.length === 0)) return "存在空白项";
  const sorted = [...trimmed].sort();
  for (let i = 0; i < 10; i++) {
    if (sorted[i] !== DIGITS_0_9[i]) {
      return "必须为 0-9 各出现一次的排列";
    }
  }
  return null;
}

type Props = {
  open: boolean;
  onClose: () => void;
  onSaved?: (mapping: MoveMapping) => void;
};

export function MoveMappingEditor({ open, onClose, onSaved }: Props) {
  const [mapping, setMapping] = useState<MoveMapping | null>(null);
  const [draft, setDraft] = useState<string[]>(() => Array(10).fill(""));
  const [loading, setLoading] = useState(false);
  const [saving, setSaving] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [info, setInfo] = useState<string | null>(null);

  useEffect(() => {
    if (!open) return;
    setError(null);
    setInfo(null);
    setLoading(true);
    invoke<MoveMapping>("get_move_mapping")
      .then((m) => {
        setMapping(m);
        setDraft([...m.digits]);
      })
      .catch((e) => setError(String(e)))
      .finally(() => setLoading(false));
  }, [open]);

  const validation = useMemo(() => validatePermutation(draft), [draft]);
  const dirty = useMemo(
    () => mapping !== null && draft.some((v, i) => v.trim() !== mapping.digits[i]),
    [draft, mapping],
  );

  if (!open) return null;

  const handleSave = async () => {
    const err = validatePermutation(draft);
    if (err) {
      setError(err);
      return;
    }
    setError(null);
    setInfo(null);
    setSaving(true);
    try {
      // 1) 先把映射写入内存（校验 + 立即生效），再弹对话框选落盘位置。
      const saved = await invoke<MoveMapping>("set_move_mapping", {
        digits: draft.map((d) => d.trim()),
      });
      setMapping(saved);
      setDraft([...saved.digits]);
      onSaved?.(saved);

      // 2) 弹原生保存对话框，默认目录 = 软件安装目录，默认文件名 move_mapping.json。
      //    保存到默认目录的同名文件下次启动会自动加载；其它位置仅作导出。
      let defaultPath = "move_mapping.json";
      try {
        const dirs = await invoke<{
          install_dir: string;
          move_mapping_filename: string;
        }>("get_default_save_paths");
        defaultPath = `${dirs.install_dir}/${dirs.move_mapping_filename}`;
      } catch {
        // 拿不到默认目录就让对话框用系统默认
      }
      const target = await dialogSave({
        title: "保存步骤映射",
        defaultPath,
        filters: [{ name: "JSON", extensions: ["json"] }],
      });
      if (!target) {
        setInfo("映射已生效（本次运行），未写入文件。");
        return;
      }
      const written = await invoke<string>("save_move_mapping_to_path", {
        path: target,
      });
      setInfo(`已保存到：${written}`);
    } catch (e) {
      setError(String(e));
    } finally {
      setSaving(false);
    }
  };

  const handleReset = async () => {
    setError(null);
    setInfo(null);
    setSaving(true);
    try {
      const reset = await invoke<MoveMapping>("reset_move_mapping");
      setMapping(reset);
      setDraft([...reset.digits]);
      setInfo("已重置为默认");
      onSaved?.(reset);
    } catch (e) {
      setError(String(e));
    } finally {
      setSaving(false);
    }
  };

  return (
    <div className="modal-overlay" onMouseDown={onClose}>
      <div className="modal-card" onMouseDown={(e) => e.stopPropagation()}>
        <div className="modal-header">
          <h2>步骤映射</h2>
          <button type="button" className="modal-close" onClick={onClose} aria-label="关闭">
            ×
          </button>
        </div>

        <p className="modal-hint">
          每条机械步骤助记符（M_L1…M_RO）对应下位机协议里的一个数字字符。
          10 项必须是 0-9 的一个排列；保存后下次启动自动加载。
        </p>

        {loading ? (
          <div className="modal-body">加载中…</div>
        ) : (
          <div className="modal-body mapping-grid">
            {(mapping?.mnemonics ?? []).map((mn, i) => (
              <label key={mn} className="mapping-row">
                <span className="mapping-mn">{mn}</span>
                <span className="mapping-arrow">→</span>
                <input
                  type="text"
                  inputMode="numeric"
                  maxLength={1}
                  className="mapping-input"
                  value={draft[i] ?? ""}
                  onChange={(e) => {
                    const v = e.target.value;
                    setDraft((prev) => {
                      const next = [...prev];
                      next[i] = v;
                      return next;
                    });
                  }}
                />
              </label>
            ))}
          </div>
        )}

        <div className="modal-status">
          {validation && <span className="msg-error">{validation}</span>}
          {error && <span className="msg-error">{error}</span>}
          {info && !error && <span className="msg-info">{info}</span>}
        </div>

        <div className="modal-actions">
          <button type="button" onClick={handleReset} disabled={saving || loading}>
            重置默认
          </button>
          <div className="modal-actions-right">
            <button type="button" onClick={onClose} disabled={saving}>
              取消
            </button>
            <button
              type="button"
              className="primary"
              onClick={handleSave}
              disabled={saving || loading || !!validation || !dirty}
            >
              {saving ? "保存中…" : "保存"}
            </button>
          </div>
        </div>
      </div>
    </div>
  );
}
