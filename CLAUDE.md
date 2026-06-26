# AeroDrag — istruzioni di repo (wheel)

Questo repo è una **chat figlia** del progetto AeroDrag, coordinata da **MOTHER**
(l'orchestratore che sta sopra tutti i repo). Sviluppa nel proprio repo, nel rispetto del contratto.

## Contratto — fonte di verità unica (NON modificare)
Il contratto d'interfaccia è `aerodrag-firmware/docs/CONTRACT.md` su `main`: è l'UNICA
fonte autorevole delle interfacce fra tutti i componenti AeroDrag. Leggilo **da git**
all'avvio (mai da copie/cache). Questo repo lo **implementa**, non lo cambia.

Vale solo la versione **RATIFICATA** su `main`. Eventuali bump **in lavorazione** (es.
seam issue aperte o draft della versione successiva) **NON sono autorevoli** finché MOTHER
non li **ratifica** su `main`: fino ad allora resta valida l'ultima versione ratificata.

I cambi d'interfaccia si **propongono via seam issue a MOTHER** — l'orchestratore che sta
**sopra tutti i repo** (ruolo/team `@.../mother`), l'unica autorizzata a **editare e
ratificare** il contratto (bump SemVer). Qualsiasi copia del contratto fuori da quel file
è "copia di lavoro, NON autorevole".
