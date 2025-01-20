# DEPLOYMENT

Una volta aperto la repository contenente tutti i codici, si avranno i seguenti file:

- **Cartella `agents`**: contiene tutti gli script standard di Carla relativi agli agenti e necessari per eseguire le simulazioni.
- **File `Adas.py`**: racchiude il codice relativo al sistema Adas in questione.
- **File `BrakeSystem.py`**: classe che racchiude la logica di frenata dei veicoli per gli script di guida manuale.
- **File `manual_control.py`**: versione modificata dell'omonimo script esempio di Carla, che implementa il sistema Adas sopradescritto.
- **File `manual_control_steeringwheel.py`**: come sopra, ma con il supporto allo sterzo.
- **File `README.md`**: contiene una copia di questo capitolo.
- **File `requirements.txt`**: contiene tutte le dipendenze necessarie per eseguire gli script in esame.
- **File `Scenari_d_uso.ipynb`**: notebook Jupyter con tutti i casi d'uso su cui è stato testato il sistema in questione.
- **File `SyncSimulation.py`**: classe Python essenziale per il corretto funzionamento dell'Adas implementata nei casi d'uso sopracitati.
- **File `wheel_config.ini`**: file di configurazione per il volante G29.

Di seguito saranno descritte le istruzioni di deployment per i vari file.

## Cartella `agents`

Nel simulatore CARLA, gli agenti sono script o moduli che controllano i veicoli autonomi per navigare nell'ambiente simulato. Nel contesto di questo lavoro, essi vengono utilizzati per le simulazioni relative agli scenari d'uso descritti in precedenza.

Normalmente, questi file sono inclusi all'interno dello zip del simulatore CARLA stesso. Tuttavia, si è scelto di copiarli nella directory principale per evitare dipendenze legate al loro percorso originale.

## `Adas.py`

Il deployment del sistema Adas si articola in due fasi principali: inizializzazione della classe e lettura dell'output. Inoltre, è possibile gestire il debug grafico per verificare il corretto funzionamento del sistema.

### Inizializzazione della classe

Tutti i concetti descritti nei capitoli precedenti sono stati implementati in una classe Python chiamata `Adas`. Questa classe può essere associata a qualsiasi veicolo tramite i seguenti parametri di inizializzazione:

- **World**: istanza del mondo del simulatore.
- **Attached_vehicle**: veicolo su cui deve essere installato il radar.
- **Get_asphalt_friction_coefficient**: lambda function per ottenere il coefficiente di attrito dell'asfalto, richiamata a ogni tick del sensore.
- **Action_listener**: lambda function invocata per attivare i freni automatici, consentendo all'Adas di adattarsi a qualsiasi logica operativa.

Oltre ai parametri obbligatori, è possibile configurare anche i seguenti campi opzionali:

- **debug**: attiva (`True`) o disattiva (`False`) la visualizzazione dei punti rilevati dal sensore.
- **visual_debug_pixel_life_time**: durata (in secondi) dei punti visualizzati.
- **visual_debug_max_point_number**: numero massimo di punti per categoria da visualizzare per ogni tick.
- **mqtt_broker**: stringa che specifica il broker MQTT.
- **mqtt_port**: porta di connessione MQTT.
- **mqtt_topic**: topic su cui pubblicare i messaggi.
- **min_fcw_state**: stato iniziale della macchina, definito dall'enum `Fcw_state`, che può assumere i valori: IDLE, WARNING, ACTION o ESCAPE.
- **vehicle_half_vertical_dimension**: altezza verticale metà veicolo (esclusa l'altezza da terra).
- **vehicle_half_horizontal_dimension**: larghezza orizzontale metà veicolo.
- **vehicle_wheelbase**: distanza interasse del veicolo.
- **min_ttc**: tempo minimo di reazione (*Time to Collision*) oltre il quale attivare i freni di emergenza o lanciare un avviso ESCAPE in caso di veicolo contromano.
- **average_reaction_time**: tempo medio di reazione umana; entro questo intervallo si possono lanciare avvisi senza attivare gli stati ACTION o ESCAPE.
- **velocity_th**: velocità minima (in m/s) per attivare l'ADAS.
- **escape_ratio_th**: percentuale minima della velocità proiettata (*projected velocity*) necessaria per attivare i freni di emergenza in condizioni ESCAPE.
- **max_radiant_steer_angle**: angolo massimo (in radianti) delle ruote durante la sterzata.
- **steer_tollerance**: tolleranza angolare per ignorare piccole rotazioni delle ruote (*dead zone*).
- **radar_range**: raggio d'azione del radar.
- **radar_height**: altezza del radar rispetto al suolo.
- **climb_inconsistencies_height_th**: altezza massima delle irregolarità in una salita (es. gradini).
- **max_slope**: pendenza massima consentita per una salita.
- **detected_point_th**: numero minimo di punti rilevati per considerare un'azione (es. invio di un warning).
- **idle_counter_th**: numero massimo di catture consecutive senza punti classificati come ESCAPE, ACTION o WARNING prima di ripristinare lo stato IDLE.

### Lettura dell'output

Dopo l'avvio del codice dell'Adas, ogni volta che vengono rilevati più di `detected_point_th` punti che richiedono una segnalazione, vengono trasmessi messaggi MQTT. Questi messaggi possono essere letti configurando un client MQTT con i seguenti parametri:

- **broker**: `broker.emqx.io`
- **port**: `1883`
- **topic**: `carla/fcw_state`

Il messaggi saranno di tre tipi e pari a ESCAPE, ACTION e WARNING.

### Debug

Come descritto in precedenza, il debug può essere configurato utilizzando i seguenti parametri:

- **debug**: attiva o disattiva la modalità di debug.
- **visual_debug_pixel_life_time**: durata (in secondi) dei punti visualizzati.
- **visual_debug_max_point_number**: numero massimo di punti visualizzati per categoria.

Impostando il parametro `debug` su `True` e configurando gli altri due parametri, ogni volta che per una determinata categoria vengono rilevati più di `detected_point_th` punti, un numero pari a `visual_debug_max_point_number` di essi (scelti casualmente) verrà visualizzato con i seguenti colori:

- **Blu**: punti appartenenti alla categoria ESCAPE.
- **Rosso**: punti appartenenti alla categoria ACTION.
- **Giallo**: punti appartenenti alla categoria WARNING.
- **Verde**: punti appartenenti alla categoria IDLE.
- **Bianco**: punti riconosciuti come parte di una salita.

I punti visualizzati resteranno sullo schermo per un tempo pari al valore impostato in `visual_debug_pixel_life_time`.

## `BrakeSystem.py`

La classe `BrakeSystem` ha lo scopo di centralizzare la logica di blocco dei veicoli nei casi di guida manuale (sia con sterzo che senza). Invocando il metodo `stop_vehicle` e passando il riferimento del veicolo, la sua accelerazione viene impostata a zero e i freni vengono attivati.

La classe include inoltre un booleano, denominato `stop_flag`, utilizzato per segnalare l'attivazione del meccanismo di arresto. Questo flag viene impostato a `TRUE` all'inizio dell'automazione e riportato a `FALSE` dopo due secondi, tramite un thread secondario che entra in modalità *sleep* per il tempo specificato.

L'idea alla base è che, negli script di guida manuale, venga introdotta una condizione per cui, se il valore del booleano è positivo (`TRUE`), i comandi dell'utente non vengano considerati. In questo modo si evita che l'intervento manuale interferisca con il meccanismo di frenata automatica.
`
## `manual_control_steeringwheel.py` e `manual_control.py`

Come anticipato, nella directory sono presenti due script per la guida manuale. Si tratta di versioni modificate degli script esempio forniti con CARLA ai quali è stato aggiunto il codice necessario per l'integrazione con il sistema ADAS e e una serie di modifiche per adattarli al contesto in esame. 

Tali modifiche consistono principalmente nella rimozione di tutti i sensori non attinenti al sistema realizzato, delle funzionalità superflue (apertura porte, accensione luci, etc) e delle funzionalità che o vanno in contrasto con lo scopo della guida manuale (guida autonoma) o con la corretta esecuzione di essa (scelta della modalità sincrona). Invece, tra le funzionalità presenti negli script originali, si è deciso di tenere la possibilità di impostare una velocità costate, settandola a 40km/h e disattivandola automaticamente quando viene attivato il sistema di frenata automatica.

È importante sottolineare che questi script sono stati progettati come semplici demo.

## `Readme.md` e `Requirements.txt`

Come anticipato, `readme.md` e `requirements.txt` svolgono una funzione di supporto: Il primo è una copia di questo capitolo, utile per fornire una descrizione generale del deployment del progetto. Il secondo contiene l'elenco di tutte le librerie necessarie per eseguire i vari script presenti nella directory.

Il file `requirements.txt` è una versione modificata dell'omonimo file presente nello zip di Carla. Per comodità, sono state aggiunte le librerie `carla`, `shapely`,  `paho-mqtt` e `networkx`, consentendo così di installare tutte le dipendenze con un unico comando:

```
python install -r requirements.txt
```

## `Scenari_d_uso.ipynb`

Il notebook raccoglie tutti gli scenari d'uso descritti in precedenza ed è suddiviso in tre blocchi principali. I primi due blocchi, *SCENARI D'USO* e *FUNZIONI DI SUPPORTO*, sono obbligatori e servono per la configurazione, mentre il terzo blocco include i 6+1 scenari menzionati.

Ogni scenario può essere eseguito singolarmente senza la necessità di aver completato gli scenari precedenti. Tuttavia, è importante notare che gli ultimi due (Scenario 5 e Scenario 6) richiedono una mappa diversa rispetto ai primi quattro; pertanto, è necessario attendere il caricamento della stessa prima di procedere.

## `SyncSimulation.py`

Il modello di radar implementato nel simulatore presenta una serie di limitazioni intrinseche.

In particolare, è stato osservato che, con alti valori di FPS, il sensore diventa pressoché inutilizzabile, generando rilevamenti spurii e velocità relative non realistiche. Sebbene la regolazione dei parametri *sensor_tick* e *points_per_second* sia necessaria per garantire il corretto funzionamento del radar al variare degli FPS, questa soluzione da sola non è sufficiente.

Per ovviare a tali problematiche, si utilizza la modalità sincrona, limitando il numero di FPS a circa 20. In questo modo, il sensore opera correttamente su qualsiasi hardware.

La classe `SyncSimulation` è progettata appositamente per gestire questa situazione: attiva e disattiva la modalità sincrona e avvia un thread che, ogni 0.05 secondi, richiama la funzione `tick` per aggiornare il simulatore.

Inoltre, nelle impostazioni della simulazione, viene modificato il parametro *fixed_delta_seconds*, impostandolo a 0.05. Questo garantisce che tra un frame e l'altro trascorra sempre lo stesso intervallo di tempo simulato, fornendo ulteriore stabilità al sistema.

Infine, negli script per la guida autonoma, oltre ad implementare `SyncSimulation`, si è dovuto anche limitare il numero degli FPS a 20.

## `wheel_config.ini`

Il file `wheel_config.ini` è un file di configurazione essenziale per l'esecuzione dello script `manual_control_steeringwheel.py`. Esso contiene i parametri necessari per settare il volante G29.