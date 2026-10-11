# SNES idle-loop survey (overnight, 2026-10-10)

**Short version:** of the 134 games on the three top lists that the core can load, **60 already get the generic skip** (median 49% of the frame skipped). **32 more need only generic fixes** (no table), **18 need a traced proof** (still no table), and **only 10 need a per-game table entry**: 8 random-seed accumulators (mostly Konami) and 2 $4212 polls. **No game changed state, picture or sound:** 0 hash mismatches in 134 games x 4,200 frames.

## Method

- Games: the 140 SNES titles of `snes_top_games_mapping.txt` (USA first, then EUR, then Japan; 6 SA-1/SuperFX titles do not load in this core).
- Per game, on the host, no input, from power-on: 4,200 frames recorded with the unmodified build (HEAD `e0b75a2`, bake on) as the reference, then replayed with the idle-skip build (working tree) with three probes: where the NMI interrupts the CPU each frame, a per-address CPU profile (frames 600-4,200), and the skip counters.
- Correctness: every state/picture/audio hash line of the idle build compared with the reference (60 or 70 lines per game).
- **Scene caveat:** title screen and attract demo only. Real play may wait in a different loop; the two real-play recordings we have (Castlevania, Zelda) wait in the same loop as their attract.

## Categories

| Cat | Meaning | Games |
|---|---|---|
| A | generic skip already takes the wait loop | 60 |
| B | idle and exact, missed for a reason with a generic fix (no table) | 32 |
| C | lap writes/calls but returns to the same state: traced proof (no table) | 18 |
| D | lap changes state every time: needs per-game arithmetic (external file) | 10 |
| E | no dominant wait loop in the scene, or one the sound CPU moves | 14 |
| X | core cannot load it (SA-1, SuperFX) | 6 |

A: skipped share of the frame, median 49%, mean 46%, range 6-92%.

### A: generic skip works now

| Game | Skipped (% of frame) | Note |
|---|---|---|
| Space Megaforce | 92 | CPU left in the loop: 100% (spans it cannot take) |
| Final Fantasy IV | 89 | CPU left in the loop: 17% (spans it cannot take) |
| Tactics Ogre | 88 |  |
| Mega Man X | 80 |  |
| Ninja Warriors | 76 |  |
| U.N. Squadron | 72 | CPU left in the loop: 22% (spans it cannot take) |
| Fatal Fury 2 | 67 |  |
| Donkey Kong Country 2: Diddy's Kong Quest | 64 |  |
| Top Gear 2 | 63 |  |
| Donkey Kong Country | 62 |  |
| Tetris Battle Gaiden | 62 |  |
| Super Mario All-Stars | 61 |  |
| Ogre Battle: The March of the Black Queen | 58 | CPU left in the loop: 53% (spans it cannot take) |
| Street Fighter II | 58 | CPU left in the loop: 24% (spans it cannot take) |
| Mega Man 7 | 57 | CPU left in the loop: 38% (spans it cannot take) |
| The Legend of Zelda: A Link to the Past | 55 | CPU left in the loop: 41% (spans it cannot take) |
| Killer Instinct (1994) | 55 | CPU left in the loop: 22% (spans it cannot take) |
| Super Street Fighter II: The New Challengers | 52 |  |
| Final Fantasy Mystic Quest | 52 | CPU left in the loop: 25% (spans it cannot take) |
| Uncharted Waters New Horizons | 52 | CPU left in the loop: 21% (spans it cannot take) |
| Wild Guns | 52 | CPU left in the loop: 22% (spans it cannot take) |
| Jungle Strike | 51 |  |
| Street Fighter II Turbo | 51 | CPU left in the loop: 17% (spans it cannot take) |
| Fire Emblem: Genealogy of the Holy War | 51 | CPU left in the loop: 23% (spans it cannot take) |
| Lufia & the Fortress of Doom | 51 |  |
| Metal Marines | 51 |  |
| Mega Man X3 | 50 | CPU left in the loop: 36% (spans it cannot take) |
| Yoshi's Cookie | 50 |  |
| Madden NFL '94 | 50 |  |
| Trials of Mana | 49 | CPU left in the loop: 39% (spans it cannot take) |
| The Death and Return of Superman | 49 |  |
| Alien 3 | 47 |  |
| Pocky & Rocky | 47 | CPU left in the loop: 30% (spans it cannot take) |
| Super Double Dragon | 46 |  |
| EVO: Search for Eden | 45 |  |
| Pocky & Rocky 2 | 44 | CPU left in the loop: 19% (spans it cannot take) |
| NBA Jam [1993] | 44 |  |
| Kirby's Dream Course | 42 | CPU left in the loop: 20% (spans it cannot take) |
| Pilotwings | 42 |  |
| F-Zero | 41 | CPU left in the loop: 22% (spans it cannot take) |
| Wario's Woods | 40 |  |
| Super Bomberman 2 | 40 |  |
| Donkey Kong Country 3 | 40 |  |
| Super Mario World | 40 | CPU left in the loop: 23% (spans it cannot take) |
| Super Star Wars | 40 | CPU left in the loop: 16% (spans it cannot take) |
| Mega Man X2 | 38 | CPU left in the loop: 40% (spans it cannot take) |
| Top Gear | 36 |  |
| Tecmo Super Bowl | 35 | CPU left in the loop: 18% (spans it cannot take) |
| Super Mario Kart | 31 | CPU left in the loop: 16% (spans it cannot take) |
| Mario Paint | 31 | CPU left in the loop: 34% (spans it cannot take) |
| Rock 'n Roll Racing | 29 |  |
| Joe & Mac 2: Lost in the Tropics | 28 | CPU left in the loop: 52% (spans it cannot take) |
| EarthBound | 22 |  |
| Uniracers | 14 |  |
| Super Smash TV | 12 | CPU left in the loop: 92% (spans it cannot take) |
| Super Turrican 2 | 12 |  |
| SimCity 2000 | 9 |  |
| NHL '94 | 7 |  |
| Flashback: The Quest for Identity | 7 |  |
| Mortal Kombat | 6 |  |

### B: generic fixes, no table

| Game | Why | Wait loop | NMI in loop | CPU in loop | Generic now |
|---|---|---|---|---|---|
| ActRaiser | polls $4210 (NMI flag): changes only at vblank | `a865:LDA 4210 \| a868:BPL fb` | 99.9 | 95.6 | 0.0 |
| Bust-A-Move | polls $4210 (NMI flag): changes only at vblank | `824c:LDA 4210 \| 824f:BPL fb` | 51.2 | 51.2 | 0.0 |
| Illusion of Gaia | polls $4210 (NMI flag): changes only at vblank | `804d:LDA 804210 \| 8051:BPL fa` | 100.0 | 58.1 | 0.0 |
| Soul Blazer | polls $4210 (NMI flag): changes only at vblank | `b1be:LDA 4210 \| b1c1:BPL fb` | 100.0 | 60.3 | 0.0 |
| Street Fighter Alpha 2 | polls $4210 (NMI flag): changes only at vblank | `f6fe:LDA 4210 \| f701:AND #80 \| f703:BEQ f9` | 100.0 | 99.4 | 0.0 |
| Super Tennis | polls $4210 (NMI flag): changes only at vblank | `c7b1:LDA 4210 \| c7b4:BPL fb` | 100.0 | 88.9 | 0.0 |
| Terranigma | polls $4210 (NMI flag): changes only at vblank | `8009:LDA 004210 \| 800d:BPL fa` | 99.5 | 77.9 | 0.0 |
| Another World | armed but not taken: DP/DB/flags changed after decode (re-decode the site) | `da01:LDA 1d4c \| da04:BEQ fb` | 99.7 | 86.7 | 3.4 |
| Kirby's Avalanche | armed but not taken: DP/DB/flags changed after decode (re-decode the site) | `80cd:LDA 25 \| 80cf:BEQ fc` | 100.0 | 100.0 | 0.0 |
| Out of This World | armed but not taken: DP/DB/flags changed after decode (re-decode the site) | `da01:LDA 1d4c \| da04:BEQ fb` | 99.7 | 84.7 | 4.2 |
| Populous | armed but not taken: DP/DB/flags changed after decode (re-decode the site) | `9357:LDA 51 \| 9359:BEQ fc` | 100.0 | 98.3 | 0.0 |
| Super Punch-Out!! | armed but not taken: DP/DB/flags changed after decode (re-decode the site) | `90b5:LDA 4b \| 90b7:BNE fc` | 74.6 | 61.5 | 4.3 |
| Cybernator | read-only poll with an index register (constant across laps) | `85f0:LDA 022e abs,x \| 85f3:BEQ fb` | 44.3 | 44.3 | 0.0 |
| Demon's Crest | read-only poll with an index register (constant across laps) | `8183:LDY #00 \| 8185:LDA 0083 \| 8188:BNE f1 \| 818a:LDA 0030 abs,y \| 818d:BMI 06 \| 818f:CMP ` | 91.4 | 49.9 | 0.0 |
| Front Mission | read-only poll with an index register (constant across laps) | `2587:LDA 0112 abs,x \| 258a:BEQ fb` | 59.6 | 59.6 | 0.0 |
| Final Fantasy V | read-only poll the current decoder misses (multi-site / DP or DB changed / long) | `945d:RAM \| 945f:RAM` | 98.8 | 69.3 | 0.0 |
| Final Fantasy VI | read-only poll the current decoder misses (multi-site / DP or DB changed / long) | `7206:RAM \| 7208:RAM` | 99.9 | 65.0 | 0.0 |
| Star Ocean | read-only poll the current decoder misses (multi-site / DP or DB changed / long) | `053d:RAM \| 0540:RAM` | 68.1 | 68.1 | 0.0 |
| Super Turrican | read-only poll the current decoder misses (multi-site / DP or DB changed / long) | `06c0:RAM \| 06c3:RAM` | 98.2 | 92.5 | 0.0 |
| Blackthorne | second wait loop (two sites alternate): arm several sites | `80d9:LDA 030e \| 80dc:BNE fb` | 45.5 | 30.3 | 0.1 |
| Chrono Trigger | second wait loop (two sites alternate): arm several sites [generic already takes 14%] | `3023:RAM \| 3026:RAM \| 3028:RAM` | 32.3 | 29.7 | 14.0 |
| Earthworm Jim 2 | second wait loop (two sites alternate): arm several sites [generic already takes 27%] | `e2b2:LDA 00 \| e2b4:BEQ fc` | 66.7 | 13.4 | 27.2 |
| Lost Vikings 2 | second wait loop (two sites alternate): arm several sites | `8118:LDA 031b \| 811b:BNE fb` | 30.9 | 15.9 | 0.2 |
| Lufia II: Rise of the Sinistrals | second wait loop (two sites alternate): arm several sites [generic already takes 30%] | `8b4e:CMP 40 \| 8b50:BEQ fc` | 44.2 | 49.8 | 29.7 |
| Shadowrun [SNES] | second wait loop (two sites alternate): arm several sites | `ee8e:CMP 7e1c92 \| ee92:BEQ fa` | 91.8 | 81.5 | 1.3 |
| Super Star Wars: Return of the Jedi | second wait loop (two sites alternate): arm several sites [generic already takes 34%] | `8b13:CMP 0709 \| 8b16:BEQ fb` | 40.4 | 9.0 | 34.0 |
| Super Star Wars: The Empire Strikes Back | second wait loop (two sites alternate): arm several sites [generic already takes 38%] | `9459:CMP 078e \| 945c:BEQ fb` | 45.9 | 13.6 | 38.5 |
| The Lost Vikings | second wait loop (two sites alternate): arm several sites | `8148:LDA 0333 \| 814b:BNE fb` | 31.0 | 15.8 | 0.2 |
| Contra III: The Alien Wars | SEP/REP/TCD inside the lap: decode the width change | `995a:REP #20 \| 995c:LDA 52 \| 995e:CMP #00ff \| 9961:BNE 0d` | 59.4 | 27.7 | 0.0 |
| Harvest Moon | SEP/REP/TCD inside the lap: decode the width change [generic already takes 6%] | `d80d:SEP #20 \| d80f:LDA 00 \| d811:BEQ fa` | 41.4 | 41.6 | 5.8 |
| Super Bomberman | SEP/REP/TCD inside the lap: decode the width change | `0342:SEP #20 \| 0344:LDA 0312 \| 0347:BIT #01 \| 0349:BEQ f7` | 92.0 | 25.4 | 0.0 |
| Tecmo Super Bowl III | SEP/REP/TCD inside the lap: decode the width change | `8716:SEP #20 \| 8718:LDA 0000 \| 871b:BPL f9` | 100.0 | 67.1 | 0.0 |

### C: traced proof, no table

| Game | Why | Wait loop | NMI in loop | CPU in loop | Generic now |
|---|---|---|---|---|---|
| Super Metroid | writes I/O in the lap: check side effects first [generic already takes 50%] | `8331:STA 05f3 \| 8334:REP #30 \| 8336:PLX \| 8337:RTL \| 8338:PHP \| 8339:PHB \| 833a:PHK \| 833b` | 100.0 | 21.5 | 49.5 |
| Axelay | writes / calls / indexed, same state each lap: traced proof | `8cf2:JSL 008120 \| 8cf6:LDX #0000 \| 8cf9:PHX \| 8cfa:PLD \| 8cfb:JSL 008939 \| 8cff:REP #20 \| ` | 39.1 | 38.4 | 0.1 |
| Breath of Fire | writes / calls / indexed, same state each lap: traced proof | `80b2:LDA #04 \| 80b4:STA 0081 \| 80b7:LDY #00 \| 80b9:LDA 0098 \| 80bc:BNE f1 \| 80be:LDA 0040 ` | 99.3 | 82.3 | 0.0 |
| Breath of Fire II | writes / calls / indexed, same state each lap: traced proof | `0df0:LDA #04 \| 0df2:STA 39 \| 0df4:LDX #00 \| 0df6:LDA 90 \| 0df8:BNE dd \| 0dfa:LDA 40 dp,x \|` | 99.6 | 79.6 | 0.0 |
| Disney's Aladdin | writes / calls / indexed, same state each lap: traced proof | `bc6d:STZ 032a \| bc70:LDA 0328 \| bc73:STA 0329 \| bc76:LDY #00 \| bc78:LDX 006f abs,y \| bc7b:` | 99.9 | 78.1 | 0.0 |
| Dragon Quest V | writes / calls / indexed, same state each lap: traced proof | `bece:PHP \| becf:JSR bed5 \| bed2:PLP \| bed3:BRA f9 \| bed5:REP #30 \| bed7:LDX 1d91 \| beda:PH` | 83.6 | 75.1 | 0.0 |
| Earthworm Jim | writes / calls / indexed, same state each lap: traced proof | `0159:LDA 28 \| 015b:AND #04 \| 015d:BNE 08 \| 015f:LDA #00 \| 0161:PER 0002 \| 0164:BRL 08fd \| ` | 34.5 | 16.0 | 0.0 |
| Final Fight | writes / calls / indexed, same state each lap: traced proof | `80dc:LDA 00c8 \| 80df:BNE e9 \| 80e1:LDA 0040 abs,y \| 80e4:BMI 06 \| 80e6:CMP #04 \| 80e8:BEQ ` | 95.0 | 75.8 | 0.0 |
| Goof Troop | writes / calls / indexed, same state each lap: traced proof | `80a7:LDY #00 \| 80a9:LDA 009b \| 80ac:BNE f1 \| 80ae:LDA 0050 abs,y \| 80b1:BMI 06 \| 80b3:CMP ` | 95.2 | 68.9 | 0.0 |
| Gradius III | writes / calls / indexed, same state each lap: traced proof | `9995:LDY #1f00 \| 9998:PHY \| 9999:PLD \| 999a:LDA 16 \| 999c:BEQ 08` | 51.5 | 20.6 | 0.0 |
| Metal Warriors | writes / calls / indexed, same state each lap: traced proof [generic already takes 72%] | `83e9:INC 2a` | 85.6 | 9.9 | 72.3 |
| SimCity (Nintendo) | writes / calls / indexed, same state each lap: traced proof | `9311:INC c7 \| 9313:LDA b9 \| 9315:BEQ fa` | 100.0 | 91.3 | 0.0 |
| Sparkster | writes / calls / indexed, same state each lap: traced proof | `ec3a:LDA #0000 \| ec3d:TCD \| ec3e:JSL 84bb30 \| ec42:BRA f6` | 35.8 | 28.7 | 0.0 |
| Super Castlevania IV | writes / calls / indexed, same state each lap: traced proof | `817f:JSL 028405 \| 8183:JSL 008189 \| 8187:BRA f6 \| 8189:REP #30 \| 818b:LDX #0000 \| 818e:PHX` | 65.6 | 49.7 | 0.0 |
| Super Ghouls 'N Ghosts | writes / calls / indexed, same state each lap: traced proof | `a6ab:STZ 02b6 \| a6ae:LDA #07 \| a6b0:STA 02b5 \| a6b3:LDY #00 \| a6b5:LDX 004e abs,y \| a6b8:C` | 99.9 | 58.2 | 0.0 |
| The Lion King | writes / calls / indexed, same state each lap: traced proof | `1a47:LDA 28 \| 1a49:AND #04 \| 1a4b:BNE 08 \| 1a4d:LDA #00 \| 1a4f:PER 0002 \| 1a52:BRL 08fd \| ` | 37.7 | 18.3 | 0.0 |
| The Magical Quest starring Mickey Mouse | writes / calls / indexed, same state each lap: traced proof | `baca:STZ 0281 \| bacd:LDA 027f \| bad0:STA 0280 \| bad3:LDY #00 \| bad5:LDX 0056 abs,y \| bad8:` | 100.0 | 56.6 | 0.0 |
| Zombies Ate My Neighbors | writes / calls / indexed, same state each lap: traced proof [generic already takes 72%] | `8372:INC 20` | 84.8 | 10.1 | 72.0 |

### D: per-game table (external file)

| Game | Why | Wait loop | NMI in loop | CPU in loop | Generic now |
|---|---|---|---|---|---|
| Dracula X | accumulates (LDA x / ADC / STA x every lap): needs arithmetic | `80c4:LDA #0000 \| 80c7:TCD \| 80c8:LDA 46 \| 80ca:ADC 42 \| 80cc:STA 46 \| 80ce:BRA f4` | 96.7 | 79.5 | 0.0 |
| International SuperStar Soccer | accumulates (LDA x / ADC / STA x every lap): needs arithmetic | `80ef:LDA #0000 \| 80f2:TCD \| 80f3:LDA 78 \| 80f5:ADC 3a \| 80f7:STA 78 \| 80f9:BRA f4` | 92.0 | 25.7 | 0.0 |
| International Superstar Soccer Deluxe | accumulates (LDA x / ADC / STA x every lap): needs arithmetic | `80d4:LDA #0000 \| 80d7:TCD \| 80d8:LDA 78 \| 80da:ADC 3a \| 80dc:STA 78 \| 80de:BRA f4` | 94.4 | 66.8 | 0.6 |
| Legend of the Mystical Ninja | accumulates (LDA x / ADC / STA x every lap): needs arithmetic | `80b4:JSL 009d27 \| 80b8:LDA #0000 \| 80bb:TCD \| 80bc:LDA 94 \| 80be:ADC 3a \| 80c0:STA 94 \| 80` | 42.3 | 33.0 | 0.0 |
| Teenage Mutant Ninja Turtles IV: Turtles in Time | accumulates (LDA x / ADC / STA x every lap): needs arithmetic | `8154:JSL 009d16 \| 8158:LDA #0000 \| 815b:TCD \| 815c:LDA 94 \| 815e:ADC 3a \| 8160:STA 94 \| 81` | 52.4 | 37.6 | 0.0 |
| Tetris & Dr. Mario | accumulates (LDA x / ADC / STA x every lap): needs arithmetic | `8dad:REP #20 \| 8daf:LDA 9e \| 8db1:ASL \| 8db2:ASL \| 8db3:CLC \| 8db4:ADC 9e \| 8db6:CLC \| 8db` | 72.9 | 54.7 | 0.0 |
| The Adventures of Batman & Robin [SNES] | accumulates (LDA x / ADC / STA x every lap): needs arithmetic | `80ed:LDA 88 \| 80ef:ADC 3a \| 80f1:STA 88 \| 80f3:BRA f8` | 99.6 | 87.6 | 0.0 |
| Tiny Toon Adventures: Buster Busts Loose | accumulates (LDA x / ADC / STA x every lap): needs arithmetic | `80c1:JSL 809aa9 \| 80c5:LDA #0000 \| 80c8:TCD \| 80c9:LDA 98 \| 80cb:ADC 3a \| 80cd:STA 98 \| 80` | 56.3 | 38.2 | 0.0 |
| Ken Griffey Jr.'s Winning Run | polls $4212 (H/V blank): value moves with the dot | `80d6:STA 22 \| 80d8:STZ 420b \| 80db:SEP #20 \| 80dd:LDA #80 \| 80df:BIT 4212 \| 80e2:BEQ fb \| ` | 99.8 | 55.2 | 0.0 |
| Live-a-Live | polls $4212 (H/V blank): value moves with the dot | `b927:BIT 4212 \| b92a:BNE fb \| b92c:BIT 4212 \| b92f:BEQ fb` | 73.9 | 73.9 | 0.0 |

### E: nothing to skip in this scene

| Game | Why | Wait loop | NMI in loop | CPU in loop | Generic now |
|---|---|---|---|---|---|
| Front Mission: Gun Hazard | no dominant wait loop in the scene (NMI lands in game code) | `` |  |  | 0.0 |
| Ken Griffey Jr. Presents | no dominant wait loop in the scene (NMI lands in game code) | `` |  |  | 0.0 |
| Mortal Kombat 3 | no NMI in the scene and no dominant loop in the CPU profile | `` |  |  | 0.0 |
| Mortal Kombat II | no NMI in the scene and no dominant loop in the CPU profile | `` |  |  | 0.1 |
| Panel de Pon | no dominant wait loop in the scene (NMI lands in game code) | `` |  |  | 0.0 |
| R-Type III: The Third Lightning | no dominant wait loop in the scene (NMI lands in game code) | `` |  |  | 2.8 |
| Secret of Evermore | no dominant wait loop in the scene (NMI lands in game code) | `` |  |  | 0.7 |
| Secret of Mana [1993] | no dominant wait loop in the scene (NMI lands in game code) | `` |  |  | 0.0 |
| Sunset Riders | no dominant wait loop in the scene (NMI lands in game code) | `` |  |  | 0.2 |
| X-Men: Mutant Apocalypse | no dominant wait loop in the scene (NMI lands in game code) | `` |  |  | 0.0 |
| Hagane | APU port handshake ($2140-3): the sound CPU moves it [generic already takes 22%] | `d072:LDA a5 \| d074:CMP a7 \| d076:BNE 02 \| d078:EOR #80 \| d07a:STA a7 \| d07c:STA 2140 \| d07` | 60.6 | 79.3 | 22.1 |
| Spider-Man -- Venom: Maximum Carnage | APU port handshake ($2140-3): the sound CPU moves it | `9512:PHP \| 9513:REP #30 \| 9515:PHX \| 9516:PHY \| 9517:SEP #30 \| 9519:LDA 2142 \| 951c:CMP 00` | 52.1 | 32.3 | 0.4 |
| Super Bomberman 3 | APU port handshake ($2140-3): the sound CPU moves it [generic already takes 12%] | `095f:LDA #10 \| 0961:STA 2141 \| 0964:LDA 2140 \| 0967:CMP 2140 \| 096a:BNE f8 \| 096c:CMP #aa ` | 52.0 | 58.3 | 11.5 |
| Ultimate Mortal Kombat 3 | APU port handshake ($2140-3): the sound CPU moves it | `ea63:CMP 2140 \| ea66:BNE fb` | 100.0 | 100.0 | 0.0 |

### X: not loadable

| Game | Why | Wait loop | NMI in loop | CPU in loop | Generic now |
|---|---|---|---|---|---|
| Kirby Super Star | coprocessor/mapper (sz=4194304) | `` |  |  |  |
| Kirby's Dream Land 3 | coprocessor/mapper (sz=4194304) | `` |  |  |  |
| Star Fox | coprocessor/mapper (sz=1048576) | `` |  |  |  |
| Stunt Race FX | coprocessor/mapper (sz=1048576) | `` |  |  |  |
| Super Mario RPG: Legend of the Seven Stars | coprocessor/mapper (sz=4194304) | `` |  |  |  |
| Super Mario World 2: Yoshi's Island | coprocessor/mapper (sz=2097152) | `` |  |  |  |

## How to cover each category

All of these keep the property the current skip has: a lap is skipped only
when it is proven to repeat exactly, and the host A/B (state + picture + audio
hashes against the unmodified build) is the gate for every change. None of
them needs a game name.

### B: generic fixes (no table)

| Fix | What changes | Why it stays exact |
|---|---|---|
| **Re-decode on environment change** (Another World, Out of This World, Kirby's Avalanche) | The NMI sampler re-decodes an armed site when E/M/X/DB/DP differ from the decode, and the span check only requires the parts the loop uses (DP only with a direct-page operand, DB only with an absolute one). | The decode is only the purity check; the per-lap proof is unchanged. |
| **Several armed sites** (Blackthorne, Lost Vikings 2, Lufia II, Earthworm Jim 2, Chrono Trigger) | Keep 2-4 sites (last armed first); the span test checks the active one, the NMI sample switches the active one. | Each site keeps its own proof. Cost: one more compare only when the active site misses. |
| **Code in WRAM** (Final Fantasy V and VI, Chrono Trigger) | Decode the loop bytes from WRAM when the pc is in $7E/$7F (or a WRAM mirror). | The loop is read-only and nothing else writes inside a span, so its own code cannot change under it. |
| **$4210 poll** (ActRaiser, Illusion of Gaia, Bust-A-Move) | Treat a read of $4210 as a read of a value that only changes at vblank (a span-boundary event). The read's side effect (clearing the NMI flag) is a no-op while the flag is already clear, which is the only state the loop spins in. | The value is constant inside a span; the proof compares it like WRAM. |
| **Indexed read-only poll** (Cybernator, Front Mission) | Allow `abs,X` / `abs,Y` / `dp,X` reads: X/Y are in the proven register state, so the address is the same every lap. Record the effective address at proof time. | Same as WRAM polls once the address is fixed by the proven registers. |
| **SEP/REP inside the lap** (Contra III, Harvest Moon) | Decode with the width the SEP/REP sets for the rest of the lap. | The proof already compares the flags at the head. |
| **Countdown** (`DEC/DEX/DEY + BNE`, Hagane's delay) | Not a recurrence: skip `min(remaining, laps that fit)` laps by subtracting from the register in one step. | The register is the only state; its value after k laps is closed-form. |

### C: traced proof (no table)

The loop writes memory or calls a subroutine, but a whole lap leaves the
machine exactly as it found it (Capcom's task scan rewrites the same counters;
Konami's JSL wait pushes and pops the same return address). The static
whitelist cannot see that; one traced lap can.

- Prove the lap through the C interpreter with a write log (the C bus path is
  hookable; the engine's fast paths are not, so the proof must not use them).
- The lap is idle if, at its end, registers and every byte it wrote are back to
  their values at its start, and it read no I/O register other than the ones
  treated as constant (B: $4210).
- Then skip whole laps exactly as now. The partial lap at the end is stepped
  from the stored boundary states (with their writes replayed from the log).

Cost: the proof lap is a C-interpreter lap, a few hundred cycles, at most once
or twice a frame. The per-span check is the same image compare as today.

### D: the per-game table (external file)

These laps change state every time, so no recurrence exists:

- **Accumulator** (Konami: Castlevania, Dracula X, ISS, ISS Deluxe, Mystical
  Ninja): `LDA #0 / TCD / LDA x / ADC y / STA x / BRA`. Skipping k laps means
  applying `x += y + carry` k times. Done natively, k iterations of two adds
  are tens of times cheaper than emulating them; a closed form is possible but
  not needed.
- **$4212 H/V poll** (Ken Griffey Jr.'s Winning Run, Live-a-Live): the value
  depends on the dot. The next change of the polled bit is known from the PPU
  timing, so the skip can jump to it. This could also be made generic; the
  table is the cautious first step.

Entry format proposal (one line per game, CRC32 of the ROM without copier
header):

```
# crc32   kind     bank:pc  len  params                         guard (loop bytes)
1a2b3c4d  ACCUM    80:80c4  12   x=dp:46 y=dp:42 w=16 tcd=0000  a90000 5b a546 6542 8546 80f4
```

`guard` is the loop's code: the handler compares it with the ROM before
arming, so a wrong CRC, another revision or a hack never runs a wrong skip.

### E: nothing to skip in the surveyed scene

NMI lands in game code, or the game runs without a wait loop. Real play may
differ; these are the first games to check with a gameplay recording.


## Files

- `snes_idle_survey.csv`: one row per game (CRC32, category, reason, skipped share, armed site, wait loop code, hash check).
- `snes_idle_skips_draft.txt`: the draft external-file entries (category D), keyed by CRC32, with guard bytes.
- `snes_top_games_mapping.txt`: list title -> ROM file.

## Also found

- **Another World / Out of This World** stop skipping after ~30 s: the game changes DP after the loop was decoded, the decode environment no longer matches and the NMI sampler never re-decodes (it sees the pc still in the armed loop). Fix in B (re-decode on environment change).
- Nothing in this survey touches the device numbers: it says which loops exist and that skipping them is exact; the cost of the skip has to be measured on the device, as Zelda showed.
