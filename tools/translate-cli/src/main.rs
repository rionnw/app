//! translate-cli: 将 2L solver 输出翻译为硬件电机命令。
//!
//! Usage:
//!   translate-cli "<solver output string>"
//!   echo "<solver output>" | translate-cli

use anyhow::{Context, Result};
use robo_core::{Moves, Translator};
use robo_translator::BasicTranslator;
use std::io::Read;

fn main() -> Result<()> {
    let input = get_input()?;

    if input.trim().is_empty() {
        eprintln!("CubeSolver Translate CLI - 2L 解法翻译为硬件指令");
        eprintln!();
        eprintln!("Usage:");
        eprintln!("  translate-cli \"(z1s0) y  (z1z0) U  ...\"");
        eprintln!("  echo \"(z1s0) y  (z1z0) U\" | translate-cli");
        std::process::exit(1);
    }

    let moves = Moves::from_solution_string(input.trim());
    let translator = BasicTranslator::new();
    let steps = translator.translate(&moves).context("翻译失败")?;

    println!("Hardware commands ({} ops):", steps.commands.len());
    for (i, cmd) in steps.commands.iter().enumerate() {
        println!("  {:3}. {cmd}", i + 1);
    }
    println!();
    println!("Encoded: {}", steps.encoded);

    Ok(())
}

fn get_input() -> Result<String> {
    let args: Vec<String> = std::env::args().collect();
    if args.len() > 1 {
        Ok(args[1..].join(" "))
    } else {
        // 从 stdin 读取
        let mut buf = String::new();
        std::io::stdin().read_to_string(&mut buf).context("读取 stdin 失败")?;
        Ok(buf)
    }
}
