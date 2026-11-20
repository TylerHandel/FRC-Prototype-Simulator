//! Undo everything, lose nothing (interface rule 7). Every edit snapshots the project first; a
//! robot is a few kilobytes, so snapshots are simpler and safer than inverse commands.
use crate::Project;

#[derive(Debug, Clone)]
pub struct Doc {
    pub project: Project,
    past: Vec<(String, Project)>,
    future: Vec<(String, Project)>,
    /// edits since the last save
    pub dirty: bool,
    /// bumped on every change, so views know to rebuild
    pub revision: u64,
}

const LIMIT: usize = 300;

impl Doc {
    pub fn new(project: Project) -> Self { Doc { project, past: vec![], future: vec![], dirty: false, revision: 0 } }

    /// Makes one edit as one undo step, named for the Edit menu.
    pub fn edit<T>(&mut self, what: &str, f: impl FnOnce(&mut Project) -> T) -> T {
        let before = self.project.clone();
        let out = f(&mut self.project);
        if self.project != before {
            // a run of edits to the same thing (dragging a number) is one step
            let merge = self.past.last().is_some_and(|(w, _)| w == what) && what.starts_with('~');
            if !merge {
                self.past.push((what.to_string(), before));
                if self.past.len() > LIMIT { self.past.remove(0); }
            }
            self.future.clear();
            self.dirty = true;
            self.revision += 1;
        }
        out
    }
    /// Starts a new step even if the next edit has the same name.
    pub fn seal(&mut self) { if let Some(last) = self.past.last_mut() { if last.0.starts_with('~') { last.0.insert(0, '!'); } } }
    pub fn can_undo(&self) -> bool { !self.past.is_empty() }
    pub fn can_redo(&self) -> bool { !self.future.is_empty() }
    pub fn undo_name(&self) -> Option<&str> { self.past.last().map(|(w, _)| w.trim_start_matches(['!', '~'])) }
    pub fn redo_name(&self) -> Option<&str> { self.future.last().map(|(w, _)| w.trim_start_matches(['!', '~'])) }
    pub fn undo(&mut self) {
        if let Some((w, p)) = self.past.pop() {
            let now = std::mem::replace(&mut self.project, p);
            self.future.push((w, now));
            self.dirty = true;
            self.revision += 1;
        }
    }
    pub fn redo(&mut self) {
        if let Some((w, p)) = self.future.pop() {
            let now = std::mem::replace(&mut self.project, p);
            self.past.push((w, now));
            self.dirty = true;
            self.revision += 1;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn undo_redo_and_drag_merge() {
        let mut d = Doc::new(Project::new("t"));
        d.edit("Rename", |p| p.name = "a".into());
        for i in 0..10 { d.edit("~Frame length", |p| p.robot.frame.length = i as f64); }
        assert_eq!(d.undo_name(), Some("Frame length"));
        d.undo();
        assert_eq!(d.project.name, "a");
        assert_ne!(d.project.robot.frame.length, 9.0);
        d.redo();
        assert_eq!(d.project.robot.frame.length, 9.0);
        d.undo(); d.undo();
        assert_eq!(d.project.name, "t");
        assert!(!d.can_undo());
    }
}
