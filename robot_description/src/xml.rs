//! The strict reading of XML elements that the SRDF reader and the reader of the URDF's
//! Gazebo extensions share: an element or an attribute a reader does not read stops the
//! read, so a part of the description that goes unread is never one the robot runs without.

use roxmltree::Node;

use crate::error::DescriptionError;

/// The child elements of `parent`, in the file's order.
pub(crate) fn elements<'a>(parent: Node<'a, 'a>) -> impl Iterator<Item = Node<'a, 'a>> {
    parent.children().filter(Node::is_element)
}

/// The refusal of the element `element` in `<parent>`, which reads only `reads`.
pub(crate) fn unread(
    element: &str,
    parent: &'static str,
    reads: &'static [&'static str],
) -> DescriptionError {
    DescriptionError::UnreadElement {
        element: element.to_string(),
        parent,
        reads,
    }
}

/// The child elements of `parent`, the element `<name>`, each of `reads` once, in the
/// order of `reads`. An element other than those of `reads`, one of them missing and one
/// of them given twice are refused.
pub(crate) fn each_once<'a, const N: usize>(
    parent: Node<'a, 'a>,
    name: &'static str,
    reads: &'static [&'static str; N],
) -> Result<[Node<'a, 'a>; N], DescriptionError> {
    let mut found: [Option<Node<'a, 'a>>; N] = [None; N];
    for child in elements(parent) {
        let tag = child.tag_name().name();
        let Some(slot) = reads.iter().position(|read| *read == tag) else {
            return Err(unread(tag, name, reads));
        };
        if found[slot].replace(child).is_some() {
            return Err(DescriptionError::ElementTwice {
                parent: name,
                element: reads[slot],
            });
        }
    }
    if let Some(slot) = found.iter().position(Option::is_none) {
        return Err(DescriptionError::MissingElement {
            parent: name,
            element: reads[slot],
        });
    }
    Ok(found.map(|child| child.expect("every element of reads is found")))
}

/// The attributes of one element, held to the ones it takes.
pub(crate) struct Attributes<'a> {
    element: Node<'a, 'a>,
}

/// The attributes of `element`, refused when one is neither of `required` nor of
/// `optional`.
pub(crate) fn attributes<'a>(
    element: Node<'a, 'a>,
    required: &[&str],
    optional: &[&str],
) -> Result<Attributes<'a>, DescriptionError> {
    match element.attributes().find(|attribute| {
        !required.contains(&attribute.name()) && !optional.contains(&attribute.name())
    }) {
        Some(attribute) => Err(DescriptionError::UnreadAttribute {
            element: element.tag_name().name().to_string(),
            attribute: attribute.name().to_string(),
        }),
        None => Ok(Attributes { element }),
    }
}

impl Attributes<'_> {
    pub(crate) fn required(&self, attribute: &'static str) -> Result<String, DescriptionError> {
        self.optional(attribute)
            .ok_or_else(|| DescriptionError::MissingAttribute {
                element: self.element.tag_name().name().to_string(),
                attribute,
            })
    }

    pub(crate) fn optional(&self, attribute: &str) -> Option<String> {
        self.element.attribute(attribute).map(str::to_string)
    }
}
