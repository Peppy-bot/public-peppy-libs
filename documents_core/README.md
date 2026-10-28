# documents_core

What the documents of every side of peppy share: the root key that names a
document's side and kind, the manifest that gives a document its name and
revision, and the digest of its canonical JSON.

```rust
use documents_core::{Document, Kind};
use serde::Deserialize;

const WORLD: Kind = Kind::new("simulation_schema", "world", 1);

#[derive(Deserialize)]
struct Catalogue {
    catalogue: String,
}

let document = Document::parse(text, WORLD)?;
let body: Catalogue = document.read()?;
let digest = document.digest();
```

`Document::parse` refuses a text whose root key or kind is not the kind's
before it reads another field, naming the root key and the kind it reads.
`read` gives the typed fields after the root key, a refusal naming the path of
the field. `Document::of` builds a document of a kind from a typed body, and
`to_json5` writes it back. The digest is the sha256 of the document in the
JSON Canonicalization Scheme of RFC 8785.

The value tree a text parses into (`parse`, and `tree_of` for a typed body)
holds what that scheme holds exactly. It refuses, each by its path, a key
twice in one object, NaN and the infinities, and a number beyond 2^53; a
float that names an integer reads as that integer. A text that nests objects
and arrays more than 128 levels deep is refused before the parser descends
into it, so a deep text stays within the 2 MiB stack of a tokio worker
thread; a typed body nested deeper is refused at the path of its 129th level.

A `Manifest` is `{ name, revision }`. A blank name is refused, and a
revision counts from 1 (`Revision`), so `revision: 0` is refused with the
revision to write.
