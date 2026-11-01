//! `workflow_bundle/v1`: a workflow carried out of the chat as a directory,
//! with the digest of each of its files and the lock of the versions it ran
//! with. The bundle names every other document, such as a world, only as a
//! file with a digest: nothing here reads a field of it.
//!
//! The bundle of a signed-off sweep also gives the thresholds, the work cell
//! parameters and the envelope of the sweep ([`Validation`]), and lists the
//! reports of the trials of the bundle under `trials/`, which are for people
//! only. The digest of a bundle is that of its bundle document without
//! `trials`, so a trial that adds its report keeps the digest.

use documents_core::{Digest, Document, Kind, Revision};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

use crate::lock::Lock;
use crate::names::Name;
use crate::refusal::{At, Refusal, Refusals};
use crate::sign_off::Thresholds;
use crate::work_cell::{Envelope, ParameterKind, Validation, WorkCellParameter, watch_refusals};
use crate::workflow::Workflow;

/// The kind of a bundle document.
const BUNDLE: Kind = Kind::new("workflow_schema", "workflow_bundle", 1);

/// The path of the bundle document in its directory.
pub const BUNDLE_PATH: &str = "bundle.json5";

/// The path of the workflow in its directory.
pub const WORKFLOW_PATH: &str = "workflow.json5";

/// The directory of the other files of a bundle.
pub const FILES_DIR: &str = "files/";

/// The directory of the reports of the trials of a bundle.
pub const TRIALS_DIR: &str = "trials/";

/// A file of a bundle directory, its path relative to the directory.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct BundleFile {
    pub path: String,
    pub content: String,
}

/// The workflow a bundle carries, by name, revision and digest.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BundledWorkflow {
    pub name: String,
    pub revision: Revision,
    pub digest: Digest,
}

/// A file of the bundle under `files/`, by its digest.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ListedFile {
    pub path: String,
    pub digest: Digest,
}

/// The fields of the bundle document.
#[derive(Clone, Debug, PartialEq, Serialize)]
#[serde(into = "BundleFields")]
pub struct BundleDocument {
    pub workflow: BundledWorkflow,
    pub files: Vec<ListedFile>,
    pub lock: Lock,
    /// What a signed-off sweep gives the bundle.
    pub validation: Option<Validation>,
    /// The path of the report of each trial of the bundle, under `trials/`.
    pub trials: Vec<String>,
}

/// The fields of the bundle document as written.
#[derive(Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct BundleFields {
    workflow: BundledWorkflow,
    files: Vec<ListedFile>,
    lock: Lock,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    thresholds: Option<Thresholds>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    work_cell_parameters: Option<BTreeMap<Name, WorkCellParameter>>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    envelope: Option<Envelope>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    trials: Vec<String>,
}

impl From<BundleDocument> for BundleFields {
    fn from(document: BundleDocument) -> BundleFields {
        let (thresholds, work_cell_parameters, envelope) = match document.validation {
            Some(validation) => (
                Some(validation.thresholds),
                Some(validation.work_cell_parameters),
                Some(validation.envelope),
            ),
            None => (None, None, None),
        };
        BundleFields {
            workflow: document.workflow,
            files: document.files,
            lock: document.lock,
            thresholds,
            work_cell_parameters,
            envelope,
            trials: document.trials,
        }
    }
}

impl BundleFields {
    /// The bundle document of these fields: the thresholds, the work cell
    /// parameters and the envelope all given or none, and the trials only
    /// with them.
    fn into_document(self) -> Result<BundleDocument, Refusals> {
        let validation = match (self.thresholds, self.work_cell_parameters, self.envelope) {
            (None, None, None) => None,
            (Some(thresholds), Some(parameters), Some(envelope)) => {
                Some(Validation::new(thresholds, parameters, envelope)?)
            }
            _ => {
                return Err(Refusals::one(Refusal::new(
                    At::Document,
                    "gives some of `thresholds`, `work_cell_parameters` and `envelope`; a signed-off sweep gives all three",
                )));
            }
        };
        if validation.is_none() && !self.trials.is_empty() {
            return Err(Refusals::one(Refusal::new(
                At::field("trials"),
                "lists trials, and the bundle has no envelope; a trial signs off only the bundle of a signed-off sweep",
            )));
        }
        Ok(BundleDocument {
            workflow: self.workflow,
            files: self.files,
            lock: self.lock,
            validation,
            trials: self.trials,
        })
    }
}

impl BundleDocument {
    /// Parses the text of a bundle document alone, reading none of the files
    /// it lists: the thresholds, the work cell parameters and the envelope
    /// all given or none, and the trials only with them.
    pub fn parse(text: &str) -> Result<BundleDocument, Refusals> {
        Document::parse(text, BUNDLE)
            .and_then(|document| document.read::<BundleFields>())
            .map_err(Refusals::from)
            .and_then(BundleFields::into_document)
    }

    /// The digest of the bundle: that of the bundle document without
    /// `trials`. A trial that adds its report keeps it, and any other change
    /// of the bundle document changes it.
    pub fn digest(&self) -> Digest {
        let without_trials = BundleDocument {
            trials: Vec::new(),
            ..self.clone()
        };
        Document::of(BUNDLE, &without_trials)
            .expect("a bundle document serializes to a document")
            .digest()
    }
}

/// A bundle whose files all match their digests.
#[derive(Clone, Debug, PartialEq)]
pub struct Bundle {
    pub document: BundleDocument,
    pub workflow: Workflow,
    /// Each file of `files/` by its path.
    pub files: BTreeMap<String, String>,
}

impl Bundle {
    /// Opens the files of a bundle directory. Refuses a missing bundle
    /// document or workflow, a path outside the directory or outside
    /// `files/`, a listed file that is missing or whose digest differs, a
    /// file of `files/` the bundle document does not list, and a workflow
    /// whose name, revision or digest differs from the bundle document.
    pub fn open(files: Vec<BundleFile>) -> Result<Bundle, Refusals> {
        let mut refusals = Vec::new();
        let mut by_path = BTreeMap::new();
        for file in files {
            if let Some(reason) = path_refusal(&file.path) {
                refusals.push(Refusal::new(At::field(file.path.clone()), reason));
                continue;
            }
            if by_path.insert(file.path.clone(), file.content).is_some() {
                refusals.push(Refusal::new(At::field(file.path), "appears twice"));
            }
        }
        Refusals::or_ok(refusals, ())?;
        let bundle_text = by_path
            .remove(BUNDLE_PATH)
            .ok_or_else(|| missing(BUNDLE_PATH))?;
        let workflow_text = by_path
            .remove(WORKFLOW_PATH)
            .ok_or_else(|| missing(WORKFLOW_PATH))?;
        let document = BundleDocument::parse(&bundle_text)
            .map_err(|refusals| refusals.in_file(BUNDLE_PATH))?;
        let workflow =
            Workflow::parse(&workflow_text).map_err(|refusals| refusals.in_file(WORKFLOW_PATH))?;
        let mut refusals = Vec::new();
        let named = &document.workflow;
        if named.name != workflow.manifest.name || named.revision != workflow.manifest.revision {
            refusals.push(Refusal::new(
                At::field(WORKFLOW_PATH),
                format!(
                    "is `{}` revision {}, and the bundle names `{}` revision {}",
                    workflow.manifest.name, workflow.manifest.revision, named.name, named.revision
                ),
            ));
        }
        if named.digest != workflow.digest() {
            refusals.push(Refusal::new(
                At::field(WORKFLOW_PATH),
                "does not match its digest in the bundle document",
            ));
        }
        let listed: BTreeSet<&str> = document
            .files
            .iter()
            .map(|file| file.path.as_str())
            .collect();
        for file in &document.files {
            if !file.path.starts_with(FILES_DIR) {
                refusals.push(Refusal::new(
                    At::field(BUNDLE_PATH),
                    format!(
                        "lists `{}`; it lists only the files under `{FILES_DIR}`",
                        file.path
                    ),
                ));
                continue;
            }
            match by_path.get(&file.path) {
                None => refusals.push(Refusal::new(
                    At::field(file.path.clone()),
                    "is listed in the bundle document and missing",
                )),
                Some(content) => match Digest::of_text(content) {
                    Ok(digest) if digest == file.digest => {}
                    Ok(_) => refusals.push(Refusal::new(
                        At::field(file.path.clone()),
                        "does not match its digest in the bundle document",
                    )),
                    Err(error) => refusals.push(Refusal::new(
                        At::field(file.path.clone()),
                        error.to_string(),
                    )),
                },
            }
        }
        let trials: BTreeSet<&str> = document.trials.iter().map(String::as_str).collect();
        for (index, path) in document.trials.iter().enumerate() {
            let refused = if !path.starts_with(TRIALS_DIR) {
                Some(format!(
                    "lists `{path}`; it lists only the reports under `{TRIALS_DIR}`"
                ))
            } else if document.trials[..index].contains(path) {
                Some(format!("lists `{path}` twice"))
            } else if !by_path.contains_key(path) {
                Some(format!("lists `{path}`, which is missing"))
            } else {
                None
            };
            if let Some(reason) = refused {
                refusals.push(Refusal::new(
                    At::field(format!("{BUNDLE_PATH}: trials")),
                    reason,
                ));
            }
        }
        for path in by_path.keys() {
            let in_list = match path.starts_with(TRIALS_DIR) {
                true => trials.contains(path.as_str()),
                false => listed.contains(path.as_str()),
            };
            if !in_list {
                refusals.push(Refusal::new(
                    At::field(path.clone()),
                    "is a file the bundle document does not list",
                ));
            }
        }
        if let Some(validation) = &document.validation {
            let misfits = Refusals::or_ok(validation_refusals(&workflow, validation), ());
            if let Err(misfits) = misfits {
                refusals.extend(misfits.in_file(BUNDLE_PATH));
            }
        }
        by_path.retain(|path, _| path.starts_with(FILES_DIR));
        Refusals::or_ok(
            refusals,
            Bundle {
                document,
                workflow,
                files: by_path,
            },
        )
    }

    /// The files of the bundle of `workflow` with `files` (each path under
    /// `files/`) and `lock`: the bundle that the chat exports.
    pub fn write(
        workflow: &Workflow,
        files: Vec<BundleFile>,
        lock: Lock,
    ) -> Result<Vec<BundleFile>, Refusals> {
        Bundle::written(workflow, files, lock, None)
    }

    /// The files of the bundle of a signed-off sweep: `workflow` with `files`
    /// (each path under `files/`), the `lock` of the versions that the sweep
    /// tested, and its `validation`. Refuses a parameter of `validation`
    /// whose roles or observation are not those of `workflow`.
    pub fn write_signed_off(
        workflow: &Workflow,
        files: Vec<BundleFile>,
        lock: Lock,
        validation: Validation,
    ) -> Result<Vec<BundleFile>, Refusals> {
        Refusals::or_ok(validation_refusals(workflow, &validation), ())?;
        Bundle::written(workflow, files, lock, Some(validation))
    }

    fn written(
        workflow: &Workflow,
        files: Vec<BundleFile>,
        lock: Lock,
        validation: Option<Validation>,
    ) -> Result<Vec<BundleFile>, Refusals> {
        let mut refusals = Vec::new();
        let mut seen = BTreeSet::new();
        let listed: Vec<ListedFile> = files
            .iter()
            .filter_map(|file| {
                let refused = path_refusal(&file.path)
                    .or_else(|| {
                        (!file.path.starts_with(FILES_DIR))
                            .then(|| format!("is outside `{FILES_DIR}`"))
                    })
                    .or_else(|| {
                        (!seen.insert(file.path.as_str())).then(|| "appears twice".to_owned())
                    });
                if let Some(reason) = refused {
                    refusals.push(Refusal::new(At::field(file.path.clone()), reason));
                    return None;
                }
                match Digest::of_text(&file.content) {
                    Ok(digest) => Some(ListedFile {
                        path: file.path.clone(),
                        digest,
                    }),
                    Err(error) => {
                        refusals.push(Refusal::new(
                            At::field(file.path.clone()),
                            error.to_string(),
                        ));
                        None
                    }
                }
            })
            .collect();
        Refusals::or_ok(refusals, ())?;
        let document = BundleDocument {
            workflow: BundledWorkflow {
                name: workflow.manifest.name.clone(),
                revision: workflow.manifest.revision,
                digest: workflow.digest(),
            },
            files: listed,
            lock,
            validation,
            trials: Vec::new(),
        };
        let bundle = Document::of(BUNDLE, &document)?;
        let written = [
            BundleFile {
                path: BUNDLE_PATH.to_owned(),
                content: bundle.to_json5(),
            },
            BundleFile {
                path: WORKFLOW_PATH.to_owned(),
                content: workflow.to_json5(),
            },
        ];
        Ok(written.into_iter().chain(files).collect())
    }
}

/// The refusals of the parameters of `validation` that do not fit
/// `workflow`: a role of `per_role` that is no role of the workflow, and a
/// watch parameter that reads no observation of it.
fn validation_refusals(workflow: &Workflow, validation: &Validation) -> Vec<Refusal> {
    let mut refusals = Vec::new();
    for (name, parameter) in &validation.work_cell_parameters {
        let unknown_roles = parameter
            .kind
            .per_role()
            .into_iter()
            .flatten()
            .filter(|role| !workflow.roles.contains_key(*role));
        refusals.extend(unknown_roles.map(|role| {
            Refusal::new(
                At::field(format!("work_cell_parameters.{name}.per_role")),
                format!("names `{role}`, which is no role of the workflow"),
            )
        }));
    }
    let observations = validation
        .work_cell_parameters
        .iter()
        .filter_map(|(name, parameter)| match &parameter.kind {
            ParameterKind::Watch { observation } => Some((name, observation)),
            ParameterKind::Control | ParameterKind::Distance { .. } => None,
        });
    refusals.extend(watch_refusals(
        workflow,
        "work_cell_parameters",
        observations,
    ));
    refusals
}

fn missing(path: &str) -> Refusals {
    Refusals::one(Refusal::new(At::field(path), "is missing from the bundle"))
}

/// Why `path` is no path of a bundle: absolute, not written with `/`, out of
/// the directory, or neither the bundle document, the workflow nor a file of
/// `files/`.
fn path_refusal(path: &str) -> Option<String> {
    if path.starts_with('/') || path.contains('\\') {
        return Some("is no relative path written with `/`".to_owned());
    }
    if path
        .split('/')
        .any(|part| part.is_empty() || part == "." || part == "..")
    {
        return Some("is no path inside the bundle directory".to_owned());
    }
    let known = path == BUNDLE_PATH
        || path == WORKFLOW_PATH
        || path.starts_with(FILES_DIR)
        || path.starts_with(TRIALS_DIR);
    (!known).then(|| {
        format!(
            "is no file of a bundle: `{BUNDLE_PATH}`, `{WORKFLOW_PATH}`, a file of `{FILES_DIR}` or a report of `{TRIALS_DIR}`"
        )
    })
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeMap;

    use super::*;

    fn a_workflow() -> Workflow {
        let text = serde_json::json!({
            "workflow_schema": "workflow/v1",
            "manifest": { "name": "home", "revision": 1 },
            "task": { "objective": "Go home" },
            "contracts": { "postures": "postures:v1" },
            "roles": { "arm": {} },
            "success": "true",
            "actions": { "arm": { "tree": { "id": "home", "timeout_s": 30, "call": { "member": "postures.move_to_home" } } } },
        });
        Workflow::parse(&text.to_string()).unwrap()
    }

    fn a_lock() -> Lock {
        Lock {
            contracts: BTreeMap::new(),
            peppy: "v0.35.1".into(),
            nodes: BTreeMap::new(),
        }
    }

    fn texts(refusals: Refusals) -> Vec<String> {
        refusals.iter().map(Refusal::to_string).collect()
    }

    /// A bundle takes its files by relative paths written with `/` inside
    /// its directory, each the bundle document, the workflow or a file of
    /// `files/`, each once.
    #[test]
    fn a_bundle_refuses_each_path_outside_its_files() {
        let written = Bundle::write(&a_workflow(), Vec::new(), a_lock()).unwrap();
        for (path, reason) in [
            ("/etc/passwd", "is no relative path written with `/`"),
            ("files\\world.json5", "is no relative path written with `/`"),
            (
                "files/../bundle.json5",
                "is no path inside the bundle directory",
            ),
            (
                "files/./world.json5",
                "is no path inside the bundle directory",
            ),
            (
                "files//world.json5",
                "is no path inside the bundle directory",
            ),
            ("notes.txt", "is no file of a bundle"),
        ] {
            let mut files = written.clone();
            files.push(BundleFile {
                path: path.to_owned(),
                content: "{}".into(),
            });
            let refused = texts(Bundle::open(files).unwrap_err());
            assert!(
                refused.len() == 1 && refused[0].contains(reason),
                "{path}: {refused:?}"
            );
        }
        let mut twice = written.clone();
        twice.push(written[0].clone());
        assert_eq!(
            texts(Bundle::open(twice).unwrap_err()),
            [format!("{}: appears twice", written[0].path)]
        );
    }

    /// The bundle document parses alone, without the files it lists, as
    /// the bundle that opens with them gives it.
    #[test]
    fn a_bundle_document_parses_alone_as_its_bundle_gives_it() {
        let file = BundleFile {
            path: format!("{FILES_DIR}world.json5"),
            content: "{ world: 1 }".into(),
        };
        let written = Bundle::write(&a_workflow(), vec![file], a_lock()).unwrap();
        let text = &written
            .iter()
            .find(|file| file.path == BUNDLE_PATH)
            .unwrap()
            .content;
        let alone = BundleDocument::parse(text).unwrap();
        assert_eq!(alone, Bundle::open(written).unwrap().document);
        assert!(BundleDocument::parse("{ workflow_schema: \"workflow/v1\" }").is_err());
    }

    #[test]
    fn a_bundle_writes_each_path_once() {
        let file = BundleFile {
            path: format!("{FILES_DIR}world.json5"),
            content: "{}".into(),
        };
        let refused = Bundle::write(&a_workflow(), vec![file.clone(), file], a_lock()).unwrap_err();
        assert_eq!(
            texts(refused),
            [format!("{FILES_DIR}world.json5: appears twice")]
        );
    }

    #[test]
    fn a_refusal_of_a_document_of_the_bundle_names_its_file() {
        let mut written = Bundle::write(&a_workflow(), Vec::new(), a_lock()).unwrap();
        let workflow = written
            .iter_mut()
            .find(|file| file.path == WORKFLOW_PATH)
            .unwrap();
        workflow.content = workflow.content.replacen("\"home\"", "\"Home!\"", 1);
        let refused = texts(Bundle::open(written).unwrap_err());
        assert!(
            refused
                .iter()
                .all(|text| text.starts_with(&format!("{WORKFLOW_PATH}: "))),
            "{refused:?}"
        );
    }

    #[test]
    fn a_bundle_document_lists_only_the_files_of_its_files_directory() {
        let written = Bundle::write(&a_workflow(), Vec::new(), a_lock()).unwrap();
        let bundle = written
            .iter()
            .find(|file| file.path == BUNDLE_PATH)
            .unwrap();
        let mut document: serde_json::Value = documents_core::parse(&bundle.content).unwrap();
        document["files"] =
            serde_json::json!([{ "path": WORKFLOW_PATH, "digest": "a".repeat(64) }]);
        let mut listed = written.clone();
        listed.retain(|file| file.path != BUNDLE_PATH);
        listed.push(BundleFile {
            path: BUNDLE_PATH.to_owned(),
            content: document.to_string(),
        });
        let refused = texts(Bundle::open(listed).unwrap_err());
        assert!(
            refused
                .iter()
                .any(|text| text.contains(&format!("lists `{WORKFLOW_PATH}`"))),
            "{refused:?}"
        );
    }
}
