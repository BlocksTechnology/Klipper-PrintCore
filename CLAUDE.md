# Blocks PrintCore

Resumo factual do trabalho feito até agora, cruzado com o código no commit `f684f7e` (2026-10-30). Os pontos marcados com **(a confirmar)** não foram verificados numa máquina real ou não foram ditos explicitamente.

## Objetivo e ambiente

- Software Klipper para um **print core amovível** (heat sink + hotend + ventoinha, com PCB de contactos). Uma resistência de ID na PCB diz à máquina que nozzle está montado.
- Requisito principal: a máquina **não entra em shutdown quando o print core é retirado** durante uma troca, mantendo a proteção normal do Klipper fora da troca.
- As máquinas são vendidas a clientes sem conhecimentos técnicos: as mensagens e os fluxos têm de ser simples e seguros.
- Máquina alvo: **RF50**, Raspberry Pi 5, toolhead **BTT EBB36 em CAN**, mainboard **Octopus Pro**, sensor do hotend **MAX31865**, **Beacon** (às vezes em modo de contacto) e **Happy Hare** (MMU).
- O ID entra no pino de termistor **TH0 da EBB36** (`Toolhead: EBB36_TH0`; pino PA3). A EBB36 tem seletor de hardware PT100/PT1000, por isso o tipo de sensor não é um ponto em aberto.
- Repositório de configs: `BlocksTechnology/RF50-Klipper`, branch `feat/amu-configs`. **Claude não faz push** (só leitura). Os ficheiros são copiados à mão para a máquina. As cópias de `printcore.cfg` e `print_core_change.cfg` nesse branch são mais antigas do que as desta repo.
- Ecrã LCD: repositório `BlocksTechnology/BlocksScreen`. **Só leitura.** O trabalho do LCD é do programador do LCD.

## Arquitetura

O projeto são três ficheiros. A documentação não está nesta repo.

| Parte | Papel |
|---|---|
| `printcore.py` | Módulo Klipper (`klippy/extras/`). Lê o ID, classifica o nozzle, gere o modo de troca, bloqueia comandos e movimento durante a troca, testa o RTD por SPI, bypass do ID, relatório de arranque. |
| `printcore.cfg` | Secção `[printcore]` (heater, pino do ID, tolerâncias, tabela de 14 nozzles, `swap_timeout`, `bypass_code`, etc.) e a macro `PRINT_CORE_CHANGE_DONE`. |
| `print_core_change.cfg` | Macros do fluxo: `PRINT_CORE_POSITIONS` (tabela de posições editável), `PRINT_CORE_CHANGE`, `PRINT_CORE_CHANGE_CANCEL`, `PRINT_CORE_SWAP_POS_FROM_HERE`, `PRINT_CORE_GOTO_POSITION` e auxiliares `_PRINT_CORE_*` (unload, eject, prepare, cooldown, abort, raise, verify, lock). |
| Documentação (fora da repo) | Doc técnica geral e doc para quem faz o LCD (Claude Docs, exportadas para PDF), mais uma comparação do nozzle do slicer com o print core para o `PRINT_START`. |

Comandos principais do módulo: `PRINTCORE_SWAP_START`, `PRINTCORE_SWAP_END`, `PRINTCORE_SWAP_EXTEND`, `PRINTCORE_CHECK_SENSOR` (só em troca), `PRINTCORE_GOTO_SWAP`, `PRINTCORE_ID_BYPASS`, `PRINTCORE_STATUS`, `PRINTCORE_REQUIRE`.

Fluxo de uma troca: `PRINT_CORE_CHANGE` (unload automático, vai à posição de troca, entra em modo de troca) → o utilizador troca o core → `PRINT_CORE_CHANGE_DONE` (verifica ID e sensor, volta a armar a proteção e faz `G28`). O ecrã segue a variável `phase` de `PRINT_CORE_CHANGE`: `idle`, `unloading`, `preparing`, `prepared`.

## Instalar, executar e testar

Instalar (a confirmar os caminhos exatos na máquina):
1. Copiar `printcore.py` para `klipper/klippy/extras/`.
2. Copiar `printcore.cfg` e `print_core_change.cfg` para a pasta de config e incluí-los no `printer.cfg`.
3. Garantir `[save_variables]` (já existe na repo; partilhado com o Happy Hare).
4. `FIRMWARE_RESTART`.

Requisitos: sensor `MAX31865` no aquecedor. O cabeçalho do módulo indica que **não foi testado em hardware**.

Testar:
- Até agora só **testes simulados** (objetos Klipper falsos e um simulador de macros), que **não estão na repo**; estavam no ambiente de trabalho da sessão.
- Antes de entregar qualquer `.cfg`, validá-lo com o parser de config do Klipper e compilar os templates Jinja. Falhou uma vez em produção por um comentário `{# ... #}`.
- Testes na máquina real ainda por fazer: fluxo completo com filamento, ordem das mensagens com o Happy Hare, comportamento com ID morto e com termístor avariado.

## Decisões técnicas e porquê

- **Shutdown só quando o RTD deixa de ler**, não quando o ID oscila: o ID pode ter ruído; o que protege o aquecedor é o RTD.
- **Durante a troca o polling do MAX31865 é parado** e o aquecimento fica bloqueado, para que retirar o core não dispare a proteção. É rearmado no fim, com confirmação de que chegam leituras.
- **Sem shutdown por sensor mau no fim da troca**: `PRINT_CORE_CHANGE_DONE` recusa com uma mensagem de erro e a troca fica ativa. Em modo de troca o aquecimento está bloqueado, por isso o shutdown não acrescentaria segurança.
- **Shutdowns do fluxo**: tempo limite da troca (`swap_timeout`, por omissão 1200 s), com aviso 60 s antes e extensão por `PRINTCORE_SWAP_EXTEND`. Há mais dois em `PRINTCORE_SWAP_END`, depois de religar o polling: sem leituras em 3 s, ou leitura a diferir mais de 5 °C do teste direto ao RTD.
- **Nada é assumido sobre o nozzle**: se o ID não lê, a máquina diz que não sabe e pede para declarar (`PRINTCORE_ID_BYPASS NOZZLE=<nome> CONFIRM=1`). O bypass vale só para a sessão: é limpo no início da troca e descartado no arranque.
- **Bypass por comando, não por config**: o utilizador final não edita ficheiros de config. `bypass_code` opcional faz o cliente ver só "contact support".
- **Remoção/inserção com o ID morto** é detetada pelo RTD (o RTD só lê com o core montado).
- **Relatório de arranque único**, 6 s depois do ready, para sair depois do Happy Hare (antes aparecia duas vezes, uma delas antes).
- **`PRINT_CORE_CHANGE_DONE` faz `G28`** logo a seguir ao fim da troca.
- **Sem limpeza do nozzle** na troca, com ou sem filamento carregado.
- **Unload automático** (filamento fica antes da gate) e confirmação de que não há filamento na gate nem no sensor do toolhead antes de entrar em modo de troca.
- **Posição de troca** `X251 Y250 Z50`, editável na tabela de `PRINT_CORE_POSITIONS`.
- **Filtro do ID**: debounce de 3 leituras (~0,9 s) e histerese; o tratamento de ruído foi deixado como está por agora.
- **Erros duplicados na consola** vêm do Klipper (cada nível de macro repete o erro); o módulo deduplica mensagens idênticas em 0,5 s.

## Convenções

- Falar com o utilizador em **português europeu**, direto, sem rodeios.
- **Todas as mensagens das macros e do módulo em inglês.** Dois níveis: `log_level` 0 (utilizador) e 1 (developer), definidos no `printcore.cfg`.
- Macros: só comentários `#` em linha própria (um `#` a meio do valor é cortado pelo parser; `{# #}` parte a macro). Uma macro não pode chamar-se a si própria.
- Entregar sempre **só os ficheiros alterados, com o nome final** (`printcore.py`, `printcore.cfg`, `print_core_change.cfg`) e dizer logo no início se mudou mais do que o pedido.
- Manter **docs e PDFs sempre atualizados** e reenviados juntos com os ficheiros.
- Não alterar código por iniciativa própria: o projeto é experimental e o utilizador copia os ficheiros à mão.
- Commits e pushes só a pedido do utilizador. Os textos propostos foram em inglês, no formato título + descrição **(a confirmar a convenção da repo)**.
- A repo é partilhada: RobeMartins e HugoCLSC também fazem commits, por isso fazer `git pull` antes de alterar.

## Estado

Feito (só validado em simulação):
- Leitura e classificação do ID, tabela de 14 nozzles, histerese e debounce.
- Modo de troca completo, bloqueio de comandos, MMU e movimento, testes do RTD, `PRINTCORE_CHECK_SENSOR`.
- Bypass do ID (`NOZZLE` + `CONFIRM`, `bypass_code` opcional), sessão apenas.
- Aviso e extensão do tempo limite, deduplicação de erros, relatório de arranque, rastreio de remoção/inserção pelo RTD.
- Macros de troca com tabela de posições, `PRINT_CORE_CHANGE_DONE` com `G28`.
- Duas docs e PDFs.

Falta:
- Validar tudo numa máquina real.
- Comparação do nozzle do slicer (`NOZZLE_SIZE={nozzle_diameter[0]}`) com o print core no `PRINT_START`: **proposta documentada, não aplicada**.
- Corrigir o módulo `bed_custom_bound.py` (ver abaixo), que é do utilizador e não está na repo.
- Trabalho do LCD (do programador do LCD): ler os novos campos e erros, botão de estender a troca, traduzir shutdown por termístor avariado.

Problemas conhecidos:
- O commit `f684f7e` (RobeMartins) removeu os códigos `[PC-01]` a `[PC-09]` e o comando `PRINTCORE_LOG`, que existiam em `387b1da` **(a confirmar se foi intencional)**. `printcore.cfg:63` ainda refere `PRINTCORE_LOG`.
- `printcore.py:1037`: a mensagem de hotend a aquecer em `PRINTCORE_SWAP_START` junta o texto do utilizador e o de developer numa só string, e fica ilegível.
- `printcore.py:1293`: falta um espaço, a mensagem sai como "mountedbut".
- `printcore.cfg:69-70`: o comentário diz que o bypass fica guardado, mas o código descarta-o no arranque.
- `bed_custom_bound.py` lê `MOVE_TO_PARK=False` como verdadeiro (`parser=bool` sobre a string "False"), o que levava a um parque indesejado em X0 Y500. As macros deixaram de passar esse parâmetro; o módulo em si **não foi corrigido** e também escreve em `kin.limits` diretamente. A correção só deve ser feita depois de confirmar que nada dependia do parque após o homing.
- Com o termístor avariado no arranque, a máquina não chega a ficar pronta (shutdown do MCU antes de o módulo correr). Isto vem de ler o código do Klipper de referência e **precisa de confirmação no `klippy.log`**.
- Se os sensores do Happy Hare devolverem `None`, a troca é bloqueada; ver a mensagem de aborto.
- Nunca se percebeu por que razão o utilizador não viu uma mensagem antiga de 5 s "ID not detected but temperature sensor works"; o código dela foi depois substituído.
