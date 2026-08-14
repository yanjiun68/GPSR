import rclpy
from rclpy.node import Node

from face_detection.identity_store import (
    DEFAULT_EMBEDDING_BACKEND,
    DEFAULT_EMBEDDING_DIMENSION,
    DEFAULT_EMBEDDING_MODEL,
    IdentityStore,
)
from face_detection.paths import default_guest_db_path


class GuestDatabaseNode(Node):
    """Special node: Provide the legacy database wrapper for SFace embeddings only."""

    def __init__(self):
        """Special function: Configure lazy model-dimension-safe database access."""
        super().__init__('guest_database_node')

        # Section: The real SFace feature length is supplied by first registration or query.
        self.declare_parameter('db_path', default_guest_db_path())
        self.declare_parameter('embedding_backend', DEFAULT_EMBEDDING_BACKEND)
        self.declare_parameter('embedding_model', DEFAULT_EMBEDDING_MODEL)
        self.declare_parameter('embedding_dimension', DEFAULT_EMBEDDING_DIMENSION)
        self.db_path = str(self.get_parameter('db_path').value)
        self.embedding_backend = str(self.get_parameter('embedding_backend').value)
        self.embedding_model = str(self.get_parameter('embedding_model').value)
        self.embedding_dimension = int(self.get_parameter('embedding_dimension').value)
        self.store = None
        if self.embedding_dimension > 0:
            self._ensure_store(self.embedding_dimension)
        self.get_logger().info(f'Guest DB node started. db_path={self.db_path}')

    def _ensure_store(self, embedding_dimension):
        """Special function: Allocate storage using an observed SFace feature length."""
        actual_dimension = int(embedding_dimension)
        if actual_dimension <= 0:
            raise ValueError('A non-empty SFace embedding is required to configure the database.')
        if self.embedding_dimension > 0 and self.embedding_dimension != actual_dimension:
            raise ValueError('Embedding dimension does not match the active SFace database.')
        if self.store is None:
            self.embedding_dimension = actual_dimension
            self.store = IdentityStore(
                self.db_path,
                logger=self.get_logger(),
                embedding_backend=self.embedding_backend,
                embedding_model=self.embedding_model,
                embedding_dimension=self.embedding_dimension,
            )
        return self.store

    @property
    def database(self):
        return self.store.database if self.store is not None else {}

    def load_database(self):
        if self.store is None:
            raise RuntimeError('Database dimension is unknown until an SFace embedding is received.')
        return self.store.load_database()

    def save_database(self):
        if self.store is None:
            raise RuntimeError('Database dimension is unknown until an SFace embedding is received.')
        return self.store.save_database()

    def register_guest(self, guest_name, embedding, shirt_color, attributes):
        store = self._ensure_store(len(embedding))
        return store.register_profile(
            profile_id=guest_name,
            name=guest_name,
            role='guest',
            embedding=embedding,
            shirt_color=shirt_color,
            attributes=attributes,
        )

    def recognize_face(self, embedding, threshold=0.45):
        store = self._ensure_store(len(embedding))
        match = store.recognize_profile(embedding, threshold=threshold)
        if not match['is_known']:
            return 'Unknown', match['match_score']
        return match['name'] or match['profile_id'], match['match_score']

    def reset_database(self):
        if self.store is None:
            raise RuntimeError('Set embedding_dimension before resetting the SFace database.')
        self.store.reset_database()


def main(args=None):
    rclpy.init(args=args)
    node = GuestDatabaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
