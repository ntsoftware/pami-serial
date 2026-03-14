# Eurobot 2026 - Prototype de communication série BW16 / Teensy

Ce programme implémente les routines de communication série entre les MCU BW16 et Teensy.

## Prérequis

- Compilateur `g++`
- GNU Make
- Editeur `VSCode` ou `VSCodium`
    - Pour le debug : extension `CodeLLDB`

Installation sous Windows : utiliser [Chocolatey](https://chocolatey.org/install) ou `WSL`

```shell
choco install mingw make
```

## Compilation

```shell
make
```

Exécutables produits :

- `test_bw16` : simulateur du MCU BW16
- `test_teensy` : simulateur du MCU Teensy

## Test

### BW16

```shell
./test_bw16
```

Fichiers de test :

- `test/bw16_rx` : données lues sur la liaison série du MCU BW16
- `test/bw16_tx` : données envoyées sur la liaison série du MCU BW16

### Teensy

```shell
./test_teensy
```

Fichiers de test :

- `test/teensy_rx` : données lues sur la liaison série du MCU Teensy
- `test/teensy_tx` : données envoyées sur la liaison série du MCU Teensy
