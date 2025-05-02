use core::mem;
use std::assert_matches::assert_matches;
use std::{collections::HashMap, mem::transmute, path::PathBuf};

use crate::{
    AsyncReader, AsyncWriter, DummyCrc, FileEntry, FileID, FileReader, FileType, FileWriter,
    VLFSError, VLFS,
};
use futures::executor::block_on;
use rand::Rng;
use replace_with::replace_with_or_abort;
use tokio::sync::Mutex;
use std::fmt::Debug;

#[cfg(feature = "internal_tests_use_debug_flash")]
use super::debug_flash::DebugFlash;
#[cfg(feature = "internal_tests_use_debug_flash")]
type FlashType = DebugFlash;

#[cfg(not(feature = "internal_tests_use_debug_flash"))]
use crate::MemoryFlash;
#[cfg(not(feature = "internal_tests_use_debug_flash"))]
type FlashType = MemoryFlash;

pub struct VLFSTestingHarness {
    pub vlfs: VLFS<FlashType, DummyCrc>,
    pub files: Mutex<HashMap<FileID, (FileType, Vec<u8>)>>,
    pub file_writers: HashMap<FileID, FileWriter<'static, FlashType, DummyCrc>>,
    pub file_readers: HashMap<FileID, (FileReader<'static, FlashType, DummyCrc>, usize)>, // cursor position
}

impl VLFSTestingHarness {
    pub async fn new(flash_image_path: PathBuf) -> Self {
        #[cfg(feature = "internal_tests_use_debug_flash")]
        let flash = DebugFlash::new().await;
        #[cfg(not(feature = "internal_tests_use_debug_flash"))]
        let flash = MemoryFlash::new(Some(flash_image_path));

        let mut vlfs = VLFS::new(flash, DummyCrc {});
        vlfs.init().await.unwrap();
        Self {
            vlfs,
            files: Mutex::new(HashMap::new()),
            file_writers: HashMap::new(),
            file_readers: HashMap::new(),
        }
    }

    pub async fn reinit(&mut self) {
        replace_with_or_abort(&mut self.vlfs, |old_fs| {
            for (_, file_writer) in self.file_writers.drain() {
                block_on(file_writer.close()).unwrap();
            }
            for (_, (file_reader, _)) in self.file_readers.drain() {
                block_on(file_reader.close());
            }

            let flash = old_fs.into_flash();
            VLFS::new(flash, DummyCrc {})
        });
        self.vlfs.init().await.unwrap();
    }

    pub async fn reinit_powerloss(&mut self){
        replace_with_or_abort(&mut self.vlfs, |old_fs| {
            for (_, file_writer) in self.file_writers.drain() {
                mem::forget(file_writer);
            }
            for (_, (file_reader, _)) in self.file_readers.drain() {
                mem::forget(file_reader);
            }

            let flash = old_fs.into_flash();
            VLFS::new(flash, DummyCrc {})
        });
        self.vlfs.init().await.unwrap();
    }

    pub async fn create_file(&self, file_type: FileType) -> FileID {
        let file_entry = self.vlfs.create_file(file_type).await.unwrap();
        let mut files = self.files.lock().await;
        let prev_max_file_id = files.iter().map(|file| file.0).max();
        if let Some(prev_max_file_id) = prev_max_file_id {
            assert!(file_entry.id > *prev_max_file_id)
        }
        files.insert(file_entry.id, (file_type, Vec::new()));
        file_entry.id
    }

    pub async fn open_file_for_write(&mut self, file_id: FileID) {
        if !self.file_writers.contains_key(&file_id) {
            let file_writer = self.vlfs.open_file_for_write(file_id).await.unwrap();
            self.file_writers.insert(file_id, unsafe {
                // SAFETY: The reference inside the Pin<Box<VLFS>> is guaranteed not to move out of its heap allocation
                transmute(file_writer)
            });
        }
    }

    pub async fn append_file(
        &mut self,
        file_id: FileID,
        length: usize,
    ) -> Result<(), VLFSError<impl defmt::Format + Debug + embedded_io_async::Error>> {
        let file_writer = self.file_writers.get_mut(&file_id).unwrap();

        let mut rng = rand::thread_rng();
        let mut vec = Vec::with_capacity(length);
        for _ in 0..length {
            vec.push(rng.r#gen::<u8>());
        }

        self.files
            .lock()
            .await
            .get_mut(&file_id)
            .unwrap()
            .1
            .extend_from_slice(vec.as_slice());
        file_writer.extend_from_slice(vec.as_slice()).await
    }

    pub async fn flush_file(&mut self, file_id: FileID) {
        let file_writer = self.file_writers.get_mut(&file_id).unwrap();
        file_writer.flush().await.unwrap();
    }

    pub async fn flush_all_files(&mut self) {
        for (_, file_writer) in self.file_writers.iter_mut() {
            file_writer.flush().await.unwrap();
        }
    }

    pub async fn close_write_file(&mut self, file_id: FileID) {
        let file_writer = self.file_writers.remove(&file_id).unwrap();
        file_writer.close().await.unwrap();
    }

    pub async fn open_file_for_read(&mut self, file_id: FileID) {
        if !self.file_readers.contains_key(&file_id) {
            let file_reader = self.vlfs.open_file_for_read(file_id).await.unwrap();
            self.file_readers.insert(file_id, unsafe {
                // SAFETY: The reference inside the Pin<Box<VLFS>> is guaranteed not to move out of its heap allocation
                (transmute(file_reader), 0)
            });
        }
    }

    pub async fn read_file(&mut self, file_id: FileID, length: usize) {
        let (file_reader, cursor) = self.file_readers.get_mut(&file_id).unwrap();

        let mut buffer = vec![0u8; length];
        let (buffer, _) = file_reader
            .read_slice(buffer.as_mut_slice(), length)
            .await
            .unwrap();

        let files = self.files.lock().await;
        let expected_buffer = files.get(&file_id).unwrap().1.as_slice();
        assert_eq!(buffer, &expected_buffer[*cursor..*cursor + buffer.len()]);
        *cursor += buffer.len();

        if buffer.len() < length {
            assert_eq!(*cursor, expected_buffer.len());
        } else if buffer.len() > length {
            panic!("buffer.len() > length");
        }
    }

    pub async fn close_read_file(&mut self, file_id: FileID) {
        let (file_reader, _) = self.file_readers.remove(&file_id).unwrap();
        file_reader.close().await;
    }

    pub async fn get_free_space(&mut self) -> u32 {
        self.vlfs.free().await
    }

    pub async fn verify_invariants(&mut self) {
        self.flush_all_files().await;
        let mut files = Vec::<FileEntry>::new();
        let mut files_iter = self.vlfs.files_iter(()).await;
        while let Some(file) = files_iter.next().await.unwrap() {
            files.push(file);
        }
        // files are sorted by id
        assert!(files.iter().map(|file| file.id.0).is_sorted());
        assert_eq!(files.len(), self.files.lock().await.len());

        let mut files_concurrent = Vec::<FileEntry>::new();
        let mut files_iter = self.vlfs.concurrent_files_iter(()).await;
        while let Some(file) = files_iter.next().await.unwrap() {
            files_concurrent.push(file);
        }
        // files are sorted by id
        assert!(files_concurrent.iter().map(|file| file.id.0).is_sorted());

        assert_eq!(files, files_concurrent);

        // all the files are still there & sizes are correct
        let self_files = self.files.lock().await;
        for (file_id, (file_type, content)) in self_files.iter() {
            let file_entry = files.iter().find(|file| file.id == *file_id).unwrap();
            assert_eq!(
                self.vlfs.is_file_opened(file_entry.id).await,
                self.file_readers.contains_key(file_id) || self.file_writers.contains_key(file_id)
            );
            assert_eq!(file_entry.typ, *file_type);
            assert!(self.vlfs.exists(*file_id).await.unwrap());
            assert_eq!(
                self.vlfs.get_file_size(*file_id).await.unwrap().0,
                content.len()
            );
        }
    }

    pub async fn remove_file(&self, file_id: FileID) {
        let mut files = self.files.lock().await;
        if files.contains_key(&file_id) {
            if self.file_writers.contains_key(&file_id) || self.file_readers.contains_key(&file_id)
            {
                assert_matches!(
                    self.vlfs.remove_file(file_id).await,
                    Err(VLFSError::FileInUse),
                );
            } else {
                self.vlfs.remove_file(file_id).await.unwrap();
                files.remove(&file_id);
            }
        } else {
            assert_matches!(
                self.vlfs.remove_file(file_id).await,
                Err(VLFSError::FileDoesNotExist),
            );
        }
    }

    pub async fn remove_files(&mut self, predicate: impl Fn(&FileID) -> bool) {
        self.vlfs
            .remove_files(|file_entry: &FileEntry| predicate(&file_entry.id))
            .await
            .unwrap();
        let mut files = self.files.lock().await;
        files.retain(|file_id, _| !predicate(file_id));
    }
}

impl Drop for VLFSTestingHarness {
    fn drop(&mut self) {
        for (_, file_writer) in self.file_writers.drain() {
            block_on(file_writer.close()).unwrap();
        }
        for (_, (file_reader, _)) in self.file_readers.drain() {
            block_on(file_reader.close());
        }
    }
}
